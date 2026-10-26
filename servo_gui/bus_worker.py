"""通过单一后台线程串行访问舵机总线。"""

from __future__ import annotations

import functools
import queue
import re
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable

from .bus import PWM_RESPONSE, TARGET_PWM_BY_ID, ZhonglingServoBus, parse_pwm


DEFAULT_TIMEOUT_S = 2.0
DEFAULT_SCAN_TIMEOUT_S = 15.0
DEFAULT_WRITE_TIMEOUT_S = 30.0

# 读数轮询
READING_INTERVAL_S = 0.1

# 零位标定（Step 5A）
DEFAULT_TOLERANCE = 10
STABILITY_SAMPLES = 3
STABILITY_SPAN = 5
SAMPLE_INTERVAL_S = 0.03
CALIBRATE_SAVE_WAIT_S = 0.3

# 持久写命令的回包比读命令慢得多：实测 V2.1.16STG 固件上
# PVER/PRAD/PMOD 均 ≤20ms，而 PCSM 用了 60ms，超过 bus.command() 固定的 30ms 等待，
# 会被误判成"无回包"。所有持久写改用带截止时间的分段读。
PERSISTENT_WRITE_TIMEOUT_S = 1.0
DRAIN_POLL_S = 0.02

# ID 标定（Step 5）
ID_SAVE_WAIT_S = 0.5
ID_SCAN_START = 0
ID_SCAN_END = 10

# 手册第五章 §14：PSCK「将当前位置设置为 1500 中间值」——这是唯一能重映射读数的命令。
# §19 PSMX「设置舵机最大值……将舵机调节到合适位置后发送」——它写的是行程上限这个
# 独立参数（见 §20 PCLEO 把「矫正值 1500」和「最大值 2500」并列），**不改变当前读数**。
# 实测佐证：对 002 发 #002PSMX! 之后回读仍是 2403，不是 2500。
# PSCK 是唯一能重映射读数的命令，而它只能映射成 1500。
# 目标读数不是 1500 的轴（002/003 取 2400）没有任何命令可用，只能手动旋转舵盘对位。
CALIBRATION_TARGET_PWM = 1500
CALIBRATION_PLAN: dict[int, dict[str, Any]] = {
    servo_id: {
        "command": "PSCK" if target == CALIBRATION_TARGET_PWM else None,
        "target": target,
        "manual": target != CALIBRATION_TARGET_PWM,
    }
    for servo_id, target in TARGET_PWM_BY_ID.items()
}


def _error(code: str, message: str, **extra: Any) -> dict[str, Any]:
    payload: dict[str, Any] = {"ok": False, "error": {"code": code, "message": message}}
    payload.update(extra)
    return payload


@dataclass
class _Request:
    operation: str
    args: tuple[Any, ...]
    reply: queue.Queue[dict[str, Any]] = field(default_factory=lambda: queue.Queue(maxsize=1))
    cancelled: threading.Event = field(default_factory=threading.Event)


class _SimulatedBus:
    """与真实通信层接口相同、但不访问设备的确定性假总线。

    profile 决定总线上"挂"了哪些舵机，用于在没有硬件时演练各种失败路径：
      assembled    001-007 全在线，读数比目标低 40，可演练读数与零位标定
      unstable     同上，但 PRAD 读数抖动，用于演练"关节不稳定"拒绝
      new          只有一个出厂 ID 000 的新舵机，用于演练 ID 标定
      numbered     只有一个已编号为 002 的舵机，用于演练手动指定源 ID 的改号（002 → 003）
      new-no-pcsm  同上，但 PCSM 不回包，用于演练"改号成功、开机释力失败"
      multi        总线上同时有 000 和 003，用于演练"多个舵机在线"拒绝
      empty        总线上没有舵机
    """

    PROFILES = ("assembled", "unstable", "new", "new-no-pcsm", "numbered", "multi", "empty")

    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout_s: float,
        response_wait_s: float,
        use_crlf: bool,
        profile: str = "assembled",
    ) -> None:
        del timeout_s, response_wait_s, use_crlf
        self.port = port
        self.baudrate = baudrate
        self.profile = profile
        self.max_limits: dict[int, int] = {}
        self._jitter_step = 0
        if profile in ("assembled", "unstable"):
            self.servos = {sid: target - 40 for sid, target in TARGET_PWM_BY_ID.items()}
        elif profile in ("new", "new-no-pcsm"):
            self.servos = {0: 1460}
        elif profile == "numbered":
            self.servos = {2: 2460}
        elif profile == "multi":
            self.servos = {0: 1460, 3: 2460}
        else:
            self.servos = {}

    def close(self) -> None:
        return None

    def _pwm_of(self, servo_id: int) -> int:
        pwm = self.servos[servo_id]
        if self.profile != "unstable":
            return pwm
        self._jitter_step += 1
        return pwm + (self._jitter_step % 3) * 9

    def command(self, command: str) -> str:
        match = re.fullmatch(r"#(\d{3})P([A-Z]{2,4})(\d{0,4})!", command)
        if match is None:
            return ""
        servo_id = int(match.group(1))
        verb = match.group(2)
        argument = match.group(3)
        if servo_id not in self.servos:
            return ""
        if verb == "VER":
            return f"#{servo_id:03d}PV模拟!"
        if verb == "RAD":
            return f"#{servo_id:03d}P{self._pwm_of(servo_id):04d}!"
        if verb in ("ULK", "ULR"):
            return "#OK!"
        if verb == "SCK":
            self.servos[servo_id] = 1500
            return "#OK!"
        if verb == "SMX":
            # 真机行为：只把行程上限设在当前位置，当前读数不变。
            self.max_limits[servo_id] = self.servos[servo_id]
            return "#OK!"
        if verb == "ID" and argument:
            target = int(argument)
            if target in self.servos:
                return ""
            self.servos[target] = self.servos.pop(servo_id)
            return f"#{target:03d}P!"
        if verb in ("CSM", "CSR"):
            if verb == "CSM" and self.profile == "new-no-pcsm":
                return ""
            # 实测 V2.1.16STG：回 #OK! 之后该舵机在断电重启前不再响应任何命令。
            # 模拟端也照做，否则演练不出真机的"写完就哑"。
            self.servos.pop(servo_id, None)
            return "#OK!"
        return ""

    def read_pwm(self, servo_id: int, retries: int) -> int | None:
        for _ in range(retries):
            pwm = parse_pwm(self.command(f"#{servo_id:03d}PRAD!"), servo_id)
            if pwm is not None:
                return pwm
        return None

    def release_torque(self, servo_id: int) -> str:
        return self.command(f"#{servo_id:03d}PULK!")

    def set_current_position_as_midpoint(self, servo_id: int) -> str:
        return self.command(f"#{servo_id:03d}PSCK!")

    def set_current_position_as_maximum(self, servo_id: int) -> str:
        return self.command(f"#{servo_id:03d}PSMX!")


class BusWorker:
    """让一个后台线程独占唯一的总线实例。"""

    def __init__(self, *, simulate: bool = False, profile: str = "assembled") -> None:
        self._requests: queue.Queue[_Request | None] = queue.Queue()
        if simulate:
            self._bus_factory: Callable[..., Any] = functools.partial(
                _SimulatedBus, profile=profile
            )
        else:
            self._bus_factory = ZhonglingServoBus
        self._bus: Any | None = None
        self._port: str | None = None
        self._baudrate: int | None = None
        self._found_ids: list[int] = []
        self._unknown_ids: list[int] = []
        self._tolerance = DEFAULT_TOLERANCE
        self._state_lock = threading.Lock()
        self._reading_ids: set[int] = set()
        self._readings: dict[int, dict[str, Any]] = {}
        # 实测：PCSM 回 #OK! 之后，该舵机在断电重启前不再响应任何命令。
        # 这不是故障，扫描时要区别于"未发现"，否则界面会误报掉线。
        self._pending_power_cycle: dict[int, str] = {}
        self._thread = threading.Thread(
            target=self._run,
            name="舵机总线线程",
            daemon=True,
        )
        self._thread.start()

    # ------------------------------------------------------------------ 连接

    def open_port(
        self,
        port: str,
        baudrate: int,
        timeout_s: float = DEFAULT_TIMEOUT_S,
    ) -> dict[str, Any]:
        if not isinstance(port, str) or not port.strip():
            return _error("invalid_request", "串口路径不能为空")
        if isinstance(baudrate, bool) or not isinstance(baudrate, int) or baudrate <= 0:
            return _error("invalid_request", "波特率必须是正整数")
        return self._submit("open_port", (port, baudrate), timeout_s)

    def close_port(self, timeout_s: float = DEFAULT_TIMEOUT_S) -> dict[str, Any]:
        return self._submit("close_port", (), timeout_s)

    def scan(
        self,
        start: int,
        end: int,
        timeout_s: float = DEFAULT_SCAN_TIMEOUT_S,
    ) -> dict[str, Any]:
        range_error = self._validate_range(start, end)
        if range_error is not None:
            return range_error
        return self._submit("scan", (start, end), timeout_s)

    def read_pwm(self, servo_id: int, timeout_s: float = DEFAULT_TIMEOUT_S) -> dict[str, Any]:
        id_error = self._validate_id(servo_id)
        if id_error is not None:
            return id_error
        return self._submit("read_pwm", (servo_id,), timeout_s)

    def release_torque(
        self,
        servo_id: int,
        timeout_s: float = DEFAULT_TIMEOUT_S,
    ) -> dict[str, Any]:
        id_error = self._validate_id(servo_id)
        if id_error is not None:
            return id_error
        return self._submit("release_torque", (servo_id,), timeout_s)

    def status(self, timeout_s: float = DEFAULT_TIMEOUT_S) -> dict[str, Any]:
        return self._submit("status", (), timeout_s)

    def shutdown(self, timeout_s: float = DEFAULT_TIMEOUT_S) -> None:
        self.close_port(timeout_s=timeout_s)
        self._requests.put(None)
        self._thread.join(timeout=timeout_s)

    # ------------------------------------------------------- 读数（Step 4）

    def start_reading(
        self,
        servo_id: int,
        release_torque: bool = True,
        timeout_s: float = DEFAULT_TIMEOUT_S,
    ) -> dict[str, Any]:
        id_error = self._validate_calibration_id(servo_id)
        if id_error is not None:
            return id_error
        return self._submit("start_reading", (servo_id, bool(release_torque)), timeout_s)

    def stop_reading(self, servo_id: int, timeout_s: float = DEFAULT_TIMEOUT_S) -> dict[str, Any]:
        id_error = self._validate_calibration_id(servo_id)
        if id_error is not None:
            return id_error
        return self._submit("stop_reading", (servo_id,), timeout_s)

    def readings(self) -> dict[str, Any]:
        """读快照不走队列，避免长扫描把界面刷新卡住。"""
        with self._state_lock:
            return {
                "ok": True,
                "active_ids": sorted(self._reading_ids),
                "tolerance": self._tolerance,
                "readings": {str(sid): dict(entry) for sid, entry in self._readings.items()},
            }

    def set_tolerance(self, tolerance: int) -> dict[str, Any]:
        if isinstance(tolerance, bool) or not isinstance(tolerance, int) or not 0 <= tolerance <= 500:
            return _error("invalid_request", "容差必须是 0 到 500 之间的整数")
        with self._state_lock:
            self._tolerance = tolerance
        return {"ok": True, "tolerance": tolerance}

    # --------------------------------------------------- 持久写（Step 5/5A/6）

    def set_id(
        self,
        source_id: int,
        target_id: int,
        confirm_text: str,
        auto_release_on_boot: bool = True,
        dry_run: bool = False,
        timeout_s: float = DEFAULT_WRITE_TIMEOUT_S,
    ) -> dict[str, Any]:
        if self._validate_id(source_id) is not None:
            return _error("invalid_request", "源 ID 必须是 0 到 254 的整数")
        target_error = self._validate_calibration_id(target_id)
        if target_error is not None:
            return target_error
        if source_id == target_id:
            return _error("invalid_request", "源 ID 与目标 ID 不能相同")
        return self._submit(
            "set_id",
            (source_id, target_id, confirm_text, bool(auto_release_on_boot), bool(dry_run)),
            timeout_s,
        )

    def calibrate_zero(
        self,
        servo_id: int,
        confirm_text: str,
        dry_run: bool = False,
        timeout_s: float = DEFAULT_WRITE_TIMEOUT_S,
    ) -> dict[str, Any]:
        id_error = self._validate_calibration_id(servo_id)
        if id_error is not None:
            return id_error
        return self._submit("calibrate_zero", (servo_id, confirm_text, bool(dry_run)), timeout_s)

    def startup_torque(
        self,
        servo_id: int,
        mode: str,
        confirm_text: str,
        dry_run: bool = False,
        timeout_s: float = DEFAULT_WRITE_TIMEOUT_S,
    ) -> dict[str, Any]:
        id_error = self._validate_calibration_id(servo_id)
        if id_error is not None:
            return id_error
        if mode not in ("release", "restore"):
            return _error("invalid_request", "mode 只能是 release 或 restore")
        return self._submit(
            "startup_torque", (servo_id, mode, confirm_text, bool(dry_run)), timeout_s
        )

    # ------------------------------------------------------------ 线程主体

    def _submit(
        self,
        operation: str,
        args: tuple[Any, ...],
        timeout_s: float,
    ) -> dict[str, Any]:
        if timeout_s <= 0:
            return _error("invalid_request", "超时时间必须大于零")
        if not self._thread.is_alive():
            return _error("worker_stopped", "总线工作线程已停止")
        request = _Request(operation=operation, args=args)
        self._requests.put(request)
        try:
            return request.reply.get(timeout=timeout_s)
        except queue.Empty:
            request.cancelled.set()
            return _error("timeout", "总线操作超时")

    def _run(self) -> None:
        while True:
            with self._state_lock:
                polling = bool(self._reading_ids)
            try:
                request = self._requests.get(timeout=READING_INTERVAL_S if polling else None)
            except queue.Empty:
                # 只有开启中的读数集合非空时才会走到这里；集合为空时上面是阻塞等待。
                self._poll_readings()
                continue
            if request is None:
                break
            if request.cancelled.is_set():
                continue
            try:
                handler = getattr(self, f"_do_{request.operation}")
                result = handler(request, *request.args)
            except Exception as exc:  # noqa: BLE001 - 统一转成结构化错误返回给界面
                result = _error("operation_failed", f"总线操作失败：{exc}")
            if not request.cancelled.is_set():
                request.reply.put(result)
        if self._bus is not None:
            self._bus.close()
            self._clear_connection()

    # ------------------------------------------------------------ 连接实现

    def _do_open_port(self, request: _Request, port: str, baudrate: int) -> dict[str, Any]:
        if self._bus is not None:
            return _error("already_connected", "串口已经连接，请先断开")
        bus = self._bus_factory(
            port=port,
            baudrate=baudrate,
            timeout_s=0.1,
            response_wait_s=0.03,
            use_crlf=True,
        )
        if request.cancelled.is_set():
            bus.close()
            return _error("timeout", "总线操作超时")
        self._bus = bus
        self._port = port
        self._baudrate = baudrate
        self._found_ids = []
        self._unknown_ids = []
        return {"ok": True, "status": self._status_snapshot()}

    def _do_close_port(self, request: _Request) -> dict[str, Any]:
        del request
        if self._bus is not None:
            self._bus.close()
        self._clear_connection()
        return {"ok": True, "status": self._status_snapshot()}

    def _do_scan(self, request: _Request, start: int, end: int) -> dict[str, Any]:
        if self._bus is None:
            return _error("not_connected", "尚未连接串口")
        found_ids, unknown_ids = self._scan_range(request, start, end)
        if found_ids is None:
            return _error("timeout", "总线操作超时")
        self._found_ids = found_ids
        self._unknown_ids = unknown_ids
        with self._state_lock:
            for servo_id in found_ids:
                # 又能回包了，说明这个轴已经断电重启过。
                self._pending_power_cycle.pop(servo_id, None)
            pending = dict(self._pending_power_cycle)
        return {
            "ok": True,
            "found_ids": found_ids.copy(),
            "unknown_ids": unknown_ids.copy(),
            "pending_power_cycle": pending,
        }

    def _scan_range(
        self,
        request: _Request,
        start: int,
        end: int,
    ) -> tuple[list[int] | None, list[int]]:
        found_ids: list[int] = []
        unknown_ids: list[int] = []
        for servo_id in range(start, end + 1):
            if request.cancelled.is_set():
                return None, []
            version = self._bus.command(f"#{servo_id:03d}PVER!")
            pwm = self._bus.read_pwm(servo_id, retries=1)
            destination = found_ids if version or pwm is not None else unknown_ids
            destination.append(servo_id)
        return found_ids, unknown_ids

    def _do_read_pwm(self, request: _Request, servo_id: int) -> dict[str, Any]:
        del request
        if self._bus is None:
            return _error("not_connected", "尚未连接串口")
        pwm = self._bus.read_pwm(servo_id, retries=1)
        if pwm is None:
            return _error("no_response", f"舵机 ID {servo_id:03d} 没有有效回包")
        return {"ok": True, "id": servo_id, "pwm": pwm}

    def _do_release_torque(self, request: _Request, servo_id: int) -> dict[str, Any]:
        del request
        if self._bus is None:
            return _error("not_connected", "尚未连接串口")
        response = self._bus.release_torque(servo_id)
        return {"ok": True, "id": servo_id, "response": response}

    def _do_status(self, request: _Request) -> dict[str, Any]:
        del request
        return {"ok": True, "status": self._status_snapshot()}

    # ------------------------------------------------------------ 读数实现

    def _do_start_reading(
        self,
        request: _Request,
        servo_id: int,
        release_torque: bool,
    ) -> dict[str, Any]:
        del request
        if self._bus is None:
            return _error("not_connected", "尚未连接串口")
        released = None
        if release_torque:
            # 不释力用户拧不动舵盘，所以默认开读数时先发一次 PULK。
            released = self._bus.release_torque(servo_id)
        with self._state_lock:
            self._reading_ids.add(servo_id)
        self._read_one(servo_id)
        return {"ok": True, "id": servo_id, "released": released}

    def _do_stop_reading(self, request: _Request, servo_id: int) -> dict[str, Any]:
        del request
        with self._state_lock:
            self._reading_ids.discard(servo_id)
            self._readings.pop(servo_id, None)
        # 停止读数时不发 PULR，保持可手动旋转状态。
        return {"ok": True, "id": servo_id}

    def _poll_readings(self) -> None:
        with self._state_lock:
            servo_ids = sorted(self._reading_ids)
        if self._bus is None:
            with self._state_lock:
                for servo_id in servo_ids:
                    self._readings[servo_id] = {
                        "id": servo_id,
                        "pwm": None,
                        "ok": False,
                        "message": "串口已断开",
                        "updated_at": time.time(),
                    }
            return
        for servo_id in servo_ids:
            self._read_one(servo_id)

    def _read_one(self, servo_id: int) -> None:
        pwm = self._bus.read_pwm(servo_id, retries=1) if self._bus is not None else None
        with self._state_lock:
            if servo_id not in self._reading_ids:
                return
            self._readings[servo_id] = {
                "id": servo_id,
                "pwm": pwm,
                "ok": pwm is not None,
                "message": "" if pwm is not None else "无响应",
                "updated_at": time.time(),
            }

    # -------------------------------------------------------- ID 标定实现

    def _do_set_id(
        self,
        request: _Request,
        source_id: int,
        target_id: int,
        confirm_text: str,
        auto_release_on_boot: bool,
        dry_run: bool,
    ) -> dict[str, Any]:
        if self._bus is None:
            return _error("not_connected", "尚未连接串口")
        busy = self._reject_if_reading()
        if busy is not None:
            return busy

        id_message = f"#{source_id:03d}PID{target_id:03d}!"
        boot_message = f"#{target_id:03d}PCSM!"
        planned = [id_message] + ([boot_message] if auto_release_on_boot else [])

        # 1. 总线独占校验：扫描结果必须恰好只有一个 ID，且等于源 ID。
        found_before, _ = self._scan_range(request, ID_SCAN_START, ID_SCAN_END)
        if found_before is None:
            return _error("timeout", "总线操作超时")
        if not found_before:
            return _error("no_response", "未检测到舵机，请检查接线", scan_before=found_before)
        if len(found_before) > 1:
            listed = "、".join(f"{sid:03d}" for sid in found_before)
            return _error(
                "bus_not_exclusive",
                f"总线上检测到多个舵机（{listed}），ID 标定只能单独连接一个舵机",
                scan_before=found_before,
            )
        if found_before[0] != source_id:
            return _error(
                "wrong_servo",
                f"总线上实际检测到的是 {found_before[0]:03d}，不是源 ID {source_id:03d}",
                scan_before=found_before,
            )

        # 2. 确认文本
        expected = f"SET ID {target_id:03d}"
        if (confirm_text or "").strip() != expected:
            return _error(
                "confirm_mismatch",
                f"确认文本不正确，需要输入：{expected}",
                scan_before=found_before,
            )

        if dry_run:
            return {
                "ok": True,
                "dry_run": True,
                "planned_commands": planned,
                "scan_before": found_before,
                "message": "dry-run：以上报文均未发送",
            }

        # 3. 改号，只发一次
        id_response = self._await_response(id_message)
        time.sleep(ID_SAVE_WAIT_S)
        found_after, unknown_after = self._scan_range(request, ID_SCAN_START, ID_SCAN_END)
        if found_after is None:
            return _error("timeout", "总线操作超时")
        # 改号前的扫描结果已经作废，立刻刷新，别让 /api/status 继续报旧 ID。
        self._found_ids = found_after
        self._unknown_ids = unknown_after
        id_ok = target_id in found_after and source_id not in found_after
        id_result = {
            "ok": id_ok,
            "command": id_message,
            "response": id_response,
            "scan_before": found_before,
            "scan_after": found_after,
        }
        if not id_ok:
            return {
                "ok": False,
                "error": {
                    "code": "id_write_failed",
                    "message": (
                        f"改号未通过验证：改号后扫描到 {found_after}，"
                        f"要求 {target_id:03d} 有响应且 {source_id:03d} 无响应"
                    ),
                },
                "id_write": id_result,
                "boot_release": None,
            }

        if not auto_release_on_boot:
            return {"ok": True, "id_write": id_result, "boot_release": None}

        # 4. 开机释力，只发一次。失败绝不重发 PID —— ID 已经变了。
        boot_response = self._await_response(boot_message)
        boot_ok = "#OK!" in boot_response
        boot_result = {
            "ok": boot_ok,
            "command": boot_message,
            "response": boot_response,
            "verified": False,
            "note": "命令已接受，需断电重启后人工确认；手册没有读回开机释力状态的命令",
            "goes_silent": boot_ok,
        }
        if boot_ok:
            self._mark_pending_power_cycle(target_id, "PCSM")
            return {"ok": True, "id_write": id_result, "boot_release": boot_result}
        return {
            "ok": False,
            "partial": True,
            "error": {
                "code": "boot_release_failed",
                "message": (
                    f"ID 已成功改为 {target_id:03d} 且不可撤销，但开机释力未确认"
                    f"（回包：{boot_response or '无回包'}）。"
                    f"请勿重发改号命令，到「开机释力」卡片对 {target_id:03d} 单独补发。"
                ),
            },
            "id_write": id_result,
            "boot_release": boot_result,
        }

    # ------------------------------------------------------ 零位标定实现

    def _do_calibrate_zero(
        self,
        request: _Request,
        servo_id: int,
        confirm_text: str,
        dry_run: bool,
    ) -> dict[str, Any]:
        del request
        if self._bus is None:
            return _error("not_connected", "尚未连接串口")
        busy = self._reject_if_reading()
        if busy is not None:
            return busy

        plan = CALIBRATION_PLAN[servo_id]
        target = plan["target"]
        if plan["manual"]:
            return _error(
                "manual_only",
                f"舵机 {servo_id:03d} 的零位目标是 {target}，没有命令能把当前位置映射成这个值"
                f"（PSCK 只能映射成 {CALIBRATION_TARGET_PWM}）。"
                "请到「舵机读数」页开启读数，手动旋转舵盘对到目标值。",
            )
        # 手册格式为 #NNNPSCK!，命令名本身已带 P，不要再补一个。
        message = f"#{servo_id:03d}{plan['command']}!"

        expected = f"CALIBRATE {servo_id:03d} TO {target:04d}"
        if (confirm_text or "").strip() != expected:
            return _error("confirm_mismatch", f"确认文本不正确，需要输入：{expected}")

        before = self._bus.read_pwm(servo_id, retries=2)
        if before is None:
            return _error("no_response", f"舵机 ID {servo_id:03d} 没有有效回包，无法标定")

        released = self._bus.release_torque(servo_id)

        samples: list[int] = []
        for index in range(STABILITY_SAMPLES):
            if index:
                time.sleep(SAMPLE_INTERVAL_S)
            sample = self._bus.read_pwm(servo_id, retries=2)
            if sample is None:
                return _error("no_response", f"舵机 ID {servo_id:03d} 采样时无回包")
            samples.append(sample)
        span = max(samples) - min(samples)
        if span > STABILITY_SPAN:
            return _error(
                "joint_not_stable",
                f"关节没有保持稳定：{STABILITY_SAMPLES} 次采样 {samples}，跨度 {span} 超过 {STABILITY_SPAN}",
                samples=samples,
            )

        if dry_run:
            return {
                "ok": True,
                "dry_run": True,
                "id": servo_id,
                "planned_commands": [message],
                "target": target,
                "before": samples[-1],
                "samples": samples,
                "released": released,
                "message": "dry-run：标定报文未发送",
            }

        response = self._await_response(message)
        # PSCK 改的是矫正值（坐标原点）。原点一移，舵机内部那个目标位置寄存器对应的
        # 物理位置就跟着偏；只要力矩是通的，它会立刻朝新坐标下的旧目标冲过去。
        # servo_zero.py 在 finally 里再发一次 PULK 正是为了压住这个动作，这里照做，
        # 而且要在回读之前发，把力矩接通的窗口压到最小。
        re_release = self._bus.release_torque(servo_id)
        time.sleep(CALIBRATE_SAVE_WAIT_S)
        after = self._bus.read_pwm(servo_id, retries=2)
        with self._state_lock:
            tolerance = self._tolerance
        verified = after is not None and abs(after - target) <= tolerance
        result = {
            "id": servo_id,
            "command": message,
            "response": response,
            "target": target,
            "before": samples[-1],
            "samples": samples,
            "after": after,
            "tolerance": tolerance,
            "verified": verified,
            # 结果以 PRAD 回读为准，#OK! 不作为最终验证。
            "torque_restored": False,
            "re_released": re_release,
        }
        if verified:
            return {"ok": True, **result}
        return {
            "ok": False,
            "error": {
                "code": "pwm_out_of_tolerance",
                "message": (
                    f"回读 {after if after is not None else '无响应'} 与目标 {target} "
                    f"相差超过 ±{tolerance}，标定未通过"
                ),
            },
            **result,
        }

    # ------------------------------------------------------ 开机释力实现

    def _do_startup_torque(
        self,
        request: _Request,
        servo_id: int,
        mode: str,
        confirm_text: str,
        dry_run: bool,
    ) -> dict[str, Any]:
        del request
        if self._bus is None:
            return _error("not_connected", "尚未连接串口")
        busy = self._reject_if_reading()
        if busy is not None:
            return busy

        verb = "PCSM" if mode == "release" else "PCSR"
        message = f"#{servo_id:03d}{verb}!"
        expected = f"{verb} {servo_id:03d}"
        if (confirm_text or "").strip() != expected:
            return _error("confirm_mismatch", f"确认文本不正确，需要输入：{expected}")

        if self._bus.read_pwm(servo_id, retries=2) is None:
            return _error("no_response", f"舵机 ID {servo_id:03d} 不在线，拒绝写入")

        if dry_run:
            return {
                "ok": True,
                "dry_run": True,
                "id": servo_id,
                "mode": mode,
                "planned_commands": [message],
                "message": "dry-run：报文未发送",
            }

        response = self._await_response(message)
        accepted = "#OK!" in response
        payload = {
            "id": servo_id,
            "mode": mode,
            "command": message,
            "response": response,
            "verified": False,
            "note": "命令已接受，需断电重启后人工确认；手册没有读回开机释力状态的命令",
            "goes_silent": accepted,
        }
        if accepted:
            self._mark_pending_power_cycle(servo_id, verb)
            return {"ok": True, **payload}
        return {
            "ok": False,
            "error": {
                "code": "not_acknowledged",
                "message": f"舵机未返回 #OK!（回包：{response or '无回包'}）",
            },
            **payload,
        }

    # ---------------------------------------------------------------- 工具

    def _await_response(
        self,
        message: str,
        timeout_s: float = PERSISTENT_WRITE_TIMEOUT_S,
    ) -> str:
        """发一条命令并等到收齐一帧（或超时），而不是固定 sleep 30ms 后读一次。

        bus.command() 的固定等待对读命令够用，对持久写不够。这里不改 bus.py，
        直接复用它已经打开的串口句柄和终止符，行为与 command() 一致，只是等待策略不同。
        """
        bus = self._bus
        serial_port = getattr(bus, "serial", None)
        if serial_port is None:
            # 模拟总线没有真实串口，直接走原路径。
            return bus.command(message)
        serial_port.reset_input_buffer()
        serial_port.write(f"{message}{bus.terminator}".encode("ascii"))
        serial_port.flush()
        deadline = time.monotonic() + timeout_s
        buffer = ""
        while time.monotonic() < deadline:
            time.sleep(DRAIN_POLL_S)
            waiting = serial_port.in_waiting
            if waiting:
                buffer += serial_port.read(waiting).decode("ascii", errors="ignore")
                if "!" in buffer:
                    break
        return buffer.strip()

    def _mark_pending_power_cycle(self, servo_id: int, verb: str) -> None:
        """记下"这个轴在断电重启前不会再回包"，避免界面把它当成掉线。"""
        with self._state_lock:
            self._pending_power_cycle[servo_id] = verb
            self._reading_ids.discard(servo_id)
            self._readings.pop(servo_id, None)

    def _reject_if_reading(self) -> dict[str, Any] | None:
        with self._state_lock:
            active = sorted(self._reading_ids)
        if not active:
            return None
        listed = "、".join(f"{sid:03d}" for sid in active)
        return _error(
            "reading_active",
            f"仍有舵机在读数中（{listed}），持久写入前请先停止全部读数",
        )

    def _status_snapshot(self) -> dict[str, Any]:
        with self._state_lock:
            active = sorted(self._reading_ids)
        with self._state_lock:
            pending = dict(self._pending_power_cycle)
        return {
            "connected": self._bus is not None,
            "port": self._port,
            "baudrate": self._baudrate,
            "found_ids": self._found_ids.copy(),
            "unknown_ids": self._unknown_ids.copy(),
            "reading_ids": active,
            "pending_power_cycle": pending,
        }

    def _clear_connection(self) -> None:
        self._bus = None
        self._port = None
        self._baudrate = None
        self._found_ids = []
        self._unknown_ids = []
        with self._state_lock:
            self._reading_ids.clear()
            self._readings.clear()
            self._pending_power_cycle.clear()

    @staticmethod
    def _validate_id(servo_id: int) -> dict[str, Any] | None:
        if isinstance(servo_id, bool) or not isinstance(servo_id, int) or not 0 <= servo_id <= 254:
            return _error("invalid_request", "舵机 ID 必须是 0 到 254 的整数")
        return None

    @staticmethod
    def _validate_calibration_id(servo_id: int) -> dict[str, Any] | None:
        if isinstance(servo_id, bool) or not isinstance(servo_id, int) or servo_id not in CALIBRATION_PLAN:
            return _error("invalid_request", "舵机 ID 必须是 1 到 7 的整数")
        return None

    @classmethod
    def _validate_range(cls, start: int, end: int) -> dict[str, Any] | None:
        start_error = cls._validate_id(start)
        if start_error is not None:
            return start_error
        end_error = cls._validate_id(end)
        if end_error is not None:
            return end_error
        if start > end:
            return _error("invalid_request", "扫描起始 ID 不能大于结束 ID")
        return None


__all__ = ["BusWorker", "CALIBRATION_PLAN", "DEFAULT_TOLERANCE", "PWM_RESPONSE", "_SimulatedBus"]
