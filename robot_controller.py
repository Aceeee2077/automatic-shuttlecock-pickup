import time
import serial


class RobotController:
    def __init__(self, use_serial=False, port="COM3", baud_rate=115200):
        self.use_serial = use_serial
        self.serial_conn = None

        if self.use_serial:
            try:
                self.serial_conn = serial.Serial(port, baud_rate, timeout=1)
                time.sleep(2)
                print("Serial connected:", port)
            except Exception as e:
                print("Serial connection failed:", e)
                self.use_serial = False

    def send_command(self, command):
        print("Command:", command)

        if self.use_serial and self.serial_conn:
            self.serial_conn.write((command + "\n").encode("utf-8"))

    def set_speed(self, left, right):
        """差速行驶：left/right 为 PWM 速度 0~255，发给 Arduino 做 analogWrite。"""
        left = int(max(0, min(255, left)))
        right = int(max(0, min(255, right)))
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

    def close(self):
        if self.serial_conn:
            self.serial_conn.close()
