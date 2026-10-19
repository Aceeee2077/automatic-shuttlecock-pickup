import json
import time
import threading
from pathlib import Path

import serial


class RobotController:
    def __init__(self, use_serial=False, port="COM3", baud_rate=115200,
                 left_factor=1.0, right_factor=1.0):
        self.use_serial = use_serial
        self.serial_conn = None

        # 直行标定修正系数（calibrate_drive.py 生成）：
        # 左轮真实偏快时 left_factor<1，下发前乘以系数抵消轮速差
        self.left_factor = left_factor
        self.right_factor = right_factor

        # 方案 20：固件上报状态（电池电压 mV）
        self.battery_mv = None
        self.last_battery_time = None

        # 串口读取线程：持续解析固件上报行（BATT <mv> 等）
        self._read_thread = None
        self._running = False

        if self.use_serial:
            try:
                self.serial_conn = serial.Serial(port, baud_rate, timeout=1)
                time.sleep(2)
                print("Serial connected:", port)
            except Exception as e:
                print("Serial connection failed:", e)
                self.use_serial = False

            if self.use_serial:
                self._start_reader()

    def _start_reader(self):
        """启动后台串口读取线程，解析固件上报。"""
        self._running = True
        self._read_thread = threading.Thread(
            target=self._read_loop, daemon=True
        )
        self._read_thread.start()

    def _read_loop(self):
        """后台线程：逐行读取固件上报，解析 BATT <mv>。"""
        while self._running:
            try:
                line = self.serial_conn.readline().decode("utf-8").strip()
            except Exception:
                break

            if not line:
                continue

            if line.startswith("BATT "):
                try:
                    self.battery_mv = int(line.split()[1])
                    self.last_battery_time = time.time()
                except (ValueError, IndexError):
                    pass

    def get_battery_mv(self):
        """返回最近上报的电池电压（mV），从未上报或数据过旧（>5s）返回 None。"""
        if self.battery_mv is None or self.last_battery_time is None:
            return None

        if time.time() - self.last_battery_time > 5.0:
            return None

        return self.battery_mv

    def send_command(self, command):
        print("Command:", command)

        if self.use_serial and self.serial_conn:
            self.serial_conn.write((command + "\n").encode("utf-8"))

    def set_speed(self, left, right):
        """差速行驶：left/right 为 PWM 速度 0~255，发给 Arduino 做 analogWrite。

        下发前乘以直行标定修正系数，抵消左右轮真实速度差。
        """
        left = int(max(0, min(255, left * self.left_factor)))
        right = int(max(0, min(255, right * self.right_factor)))
        self.send_command(f"SPEED {left} {right}")

    def forward(self):
        self.send_command("MOVE_FORWARD")

    def backward(self):
        self.send_command("MOVE_BACKWARD")

    def turn_left(self):
        self.send_command("TURN_LEFT")

    def turn_right(self):
        self.send_command("TURN_RIGHT")

    def stop(self):
        self.send_command("STOP")

    def collect_on(self):
        self.send_command("COLLECT_ON")

    def collect_off(self):
        self.send_command("COLLECT_OFF")

    def heartbeat(self):
        """看门狗心跳：不改变当前动作，仅刷新 Arduino 端看门狗计时。"""
        if self.use_serial and self.serial_conn:
            self.serial_conn.write(b"PING\n")

    def close(self):
        self._running = False

        if self._read_thread is not None:
            self._read_thread.join(timeout=1.0)

        if self.serial_conn:
            self.serial_conn.close()
