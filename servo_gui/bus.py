"""串口舵机通信层。

来源：/home/zekeep/rebot_lerobot/tools/zhonglin_servo/servo_zero.py
搬运日期：2026-09-10
若原文件中的对应实现发生变更，本文件必须同步更新。
"""

import re
import time

import serial


PWM_RESPONSE = re.compile(r"#(?P<id>\d{3})P(?P<pwm>\d{4})!")
TARGET_PWM_BY_ID = {
    1: 1500,
    2: 2400,
    3: 2400,
    4: 1500,
    5: 1500,
    6: 1500,
    7: 1500,
}


def parse_pwm(response: str, expected_id: int) -> int | None:
    for match in PWM_RESPONSE.finditer(response):
        if int(match.group("id")) == expected_id:
            return int(match.group("pwm"))
    return None


class ZhonglingServoBus:
    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout_s: float,
        response_wait_s: float,
        use_crlf: bool,
    ) -> None:
        self.response_wait_s = response_wait_s
        self.terminator = "\r\n" if use_crlf else ""
        self.serial = serial.Serial(port, baudrate, timeout=timeout_s)

    def close(self) -> None:
        self.serial.close()

    def command(self, command: str) -> str:
        self.serial.reset_input_buffer()
        self.serial.write(f"{command}{self.terminator}".encode("ascii"))
        self.serial.flush()
        time.sleep(self.response_wait_s)
        return self.serial.read_all().decode("ascii", errors="ignore").strip()

    def read_pwm(self, servo_id: int, retries: int) -> int | None:
        for _ in range(retries):
            response = self.command(f"#{servo_id:03d}PRAD!")
            pwm = parse_pwm(response, servo_id)
            if pwm is not None:
                return pwm
        return None

    def release_torque(self, servo_id: int) -> str:
        return self.command(f"#{servo_id:03d}PULK!")

    def restore_torque(self, servo_id: int) -> str:
        return self.command(f"#{servo_id:03d}PULR!")

    def set_current_position_as_midpoint(self, servo_id: int) -> str:
        # PSCK is the Zhongling midpoint calibration command. PCSK is not.
        return self.command(f"#{servo_id:03d}PSCK!")

    def set_current_position_as_maximum(self, servo_id: int) -> str:
        # PSMX stores the current physical position as PWM 2500.
        return self.command(f"#{servo_id:03d}PSMX!")

    def set_current_position_as_startup(self, servo_id: int) -> str:
        """Persist the current physical position as the power-on position."""
        return self.command(f"#{servo_id:03d}PCSD!")
