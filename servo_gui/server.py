"""ZEKEEP指擎串口舵机标定工具的标准库 HTTP 服务。"""

from __future__ import annotations

import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from serial.tools import list_ports

from .bus_worker import CALIBRATION_PLAN, BusWorker, _SimulatedBus


INDEX_HTML = Path(__file__).with_name("static") / "index.html"
# 主臂走 USB 转串口适配器，枚举为 /dev/ttyUSB*。
# 只列这一类，既少一次误选，也避免把从臂的 CAN 适配器（/dev/ttyACM*）当成主臂连上去。
PORT_PREFIX = "/dev/ttyUSB"
# PCSD（设置上电启动位置）本工具不实现：PCSM 开机释力会让 PCSD 失效，二者互斥。
NOT_IMPLEMENTED_PATHS = {
    "/api/startup_position": "本工具不实现 PCSD：开机释力（PCSM）会让 PCSD 失效，二者互斥",
}


def _port_record(port: Any) -> dict[str, Any]:
    return {
        "device": port.device,
        "description": port.description,
        "hwid": port.hwid,
        "vid": port.vid,
        "pid": port.pid,
        "serial_number": port.serial_number,
        "manufacturer": port.manufacturer,
        "product": port.product,
        "interface": port.interface,
        "location": port.location,
    }


def _list_port_records(*, simulate: bool) -> list[dict[str, Any]]:
    records = [
        _port_record(port)
        for port in list_ports.comports()
        if port.device.startswith(PORT_PREFIX)
    ]
    if simulate:
        records.insert(
            0,
            {
                "device": "模拟串口",
                "description": "ZEKEEP指擎舵机模拟设备",
                "hwid": "SIMULATED",
                "vid": None,
                "pid": None,
                "serial_number": None,
                "manufacturer": None,
                "product": None,
                "interface": None,
                "location": None,
            },
        )
    return records


def _http_status_for(result: dict[str, Any]) -> int:
    if result.get("ok"):
        return 200
    code = result.get("error", {}).get("code")
    return {
        "invalid_request": 400,
        "not_connected": 409,
        "already_connected": 409,
        "no_response": 502,
        "timeout": 504,
        "worker_stopped": 503,
        "operation_failed": 503,
        "confirm_mismatch": 400,
        "bus_not_exclusive": 409,
        "wrong_servo": 409,
        "reading_active": 409,
        "joint_not_stable": 409,
        "manual_only": 409,
        "id_write_failed": 502,
        "boot_release_failed": 502,
        "pwm_out_of_tolerance": 502,
        "not_acknowledged": 502,
    }.get(code, 500)


def make_handler(worker: BusWorker, *, simulate: bool) -> type[BaseHTTPRequestHandler]:
    class ServoGuiHandler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            path = urlsplit(self.path).path
            if path == "/":
                self._send_index()
                return
            if path == "/api/ports":
                self._send_json(200, {"ports": _list_port_records(simulate=simulate)})
                return
            if path == "/api/status":
                result = worker.status()
                if result.get("ok"):
                    self._send_json(200, result["status"])
                else:
                    self._send_json(_http_status_for(result), result)
                return
            if path == "/api/targets":
                self._send_json(200, {
                    "plan": {
                        f"{sid:03d}": {"id": sid, **plan}
                        for sid, plan in sorted(CALIBRATION_PLAN.items())
                    },
                    "tolerance": worker.readings()["tolerance"],
                })
                return
            if path == "/api/readings":
                self._send_json(200, worker.readings())
                return
            prefix = "/api/reading/"
            if path.startswith(prefix):
                id_text = path[len(prefix) :]
                if not id_text.isdigit():
                    self._send_json(400, {"error": "舵机 ID 必须是整数"})
                    return
                result = worker.read_pwm(int(id_text, 10))
                self._send_json(_http_status_for(result), result)
                return
            self._send_json(404, {"error": "未找到接口"})

        def do_POST(self) -> None:
            path = urlsplit(self.path).path
            if path in NOT_IMPLEMENTED_PATHS:
                self._send_json(501, {"error": NOT_IMPLEMENTED_PATHS[path]})
                return
            try:
                body = self._read_json_body()
            except ValueError as exc:
                self._send_json(400, {"error": str(exc)})
                return

            if path == "/api/connect":
                result = worker.open_port(body.get("port"), body.get("baudrate"))
            elif path == "/api/disconnect":
                result = worker.close_port()
            elif path == "/api/scan":
                result = worker.scan(body.get("start", 0), body.get("end", 10))
            elif path == "/api/reading/start":
                result = worker.start_reading(
                    body.get("id"),
                    body.get("release_torque", True),
                )
            elif path == "/api/reading/stop":
                result = worker.stop_reading(body.get("id"))
            elif path == "/api/tolerance":
                result = worker.set_tolerance(body.get("tolerance"))
            elif path == "/api/set_id":
                result = worker.set_id(
                    body.get("source_id"),
                    body.get("target_id"),
                    body.get("confirm_text", ""),
                    body.get("auto_release_on_boot", True),
                    body.get("dry_run", False),
                )
            elif path == "/api/calibrate_zero":
                result = worker.calibrate_zero(
                    body.get("id"),
                    body.get("confirm_text", ""),
                    body.get("dry_run", False),
                )
            elif path == "/api/startup_torque":
                result = worker.startup_torque(
                    body.get("id"),
                    body.get("mode", ""),
                    body.get("confirm_text", ""),
                    body.get("dry_run", False),
                )
            else:
                self._send_json(404, {"error": "未找到接口"})
                return
            self._send_json(_http_status_for(result), result)

        def _read_json_body(self) -> dict[str, Any]:
            length_text = self.headers.get("Content-Length", "0")
            try:
                length = int(length_text, 10)
            except ValueError as exc:
                raise ValueError("Content-Length 无效") from exc
            if length == 0:
                return {}
            try:
                value = json.loads(self.rfile.read(length).decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ValueError("请求体必须是有效的 JSON") from exc
            if not isinstance(value, dict):
                raise ValueError("JSON 请求体必须是对象")
            return value

        def _send_json(self, status: int, payload: dict[str, Any]) -> None:
            data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _send_index(self) -> None:
            try:
                data = INDEX_HTML.read_bytes()
            except OSError as exc:
                self._send_json(500, {"error": f"无法读取界面文件：{exc}"})
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, format: str, *args: Any) -> None:
            super().log_message(format, *args)

    return ServoGuiHandler


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ZEKEEP指擎串口舵机标定工具后端")
    parser.add_argument("--port", type=int, default=8770, help="HTTP 监听端口")
    parser.add_argument("--simulate", action="store_true", help="使用模拟舵机，不打开真实串口")
    parser.add_argument(
        "--simulate-profile",
        default="assembled",
        choices=_SimulatedBus.PROFILES,
        help="模拟总线上挂的舵机组合，仅在 --simulate 时生效",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    worker = BusWorker(simulate=args.simulate, profile=args.simulate_profile)
    server = ThreadingHTTPServer(
        ("127.0.0.1", args.port),
        make_handler(worker, simulate=args.simulate),
    )
    mode = f"模拟模式 · {args.simulate_profile}" if args.simulate else "真实设备模式"
    print(f"ZEKEEP指擎舵机工具后端已启动：http://127.0.0.1:{args.port}（{mode}）", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n服务已停止。")
    finally:
        server.server_close()
        worker.shutdown()


if __name__ == "__main__":
    main()
