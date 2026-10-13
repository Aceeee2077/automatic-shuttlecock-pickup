/*
  Automatic Shuttlecock Pickup Machine
  Arduino / ESP32 底层控制示例

  Python 通过串口发送指令：
  MOVE_FORWARD
  MOVE_BACKWARD
  TURN_LEFT
  TURN_RIGHT
  STOP
  COLLECT_ON
    COLLECT_OFF
    SPEED <left_pwm> <right_pwm>   比例控制差速行驶（0~255）
    PING                           仅刷新看门狗，不改变动作（捡球期间发）

  串口看门狗（方案 6）：
  超过 WATCHDOG_TIMEOUT_MS 没收到任何指令就自动停车，
  防止 Python 端崩溃/卡死/拔线后小车失控。
  任何指令都会刷新看门狗（SPEED 每帧发送，天然作为心跳）。
*/

const int LEFT_MOTOR_IN1 = 5;   // PWM
const int LEFT_MOTOR_IN2 = 6;   // PWM

const int RIGHT_MOTOR_IN1 = 9;  // PWM
const int RIGHT_MOTOR_IN2 = 10; // PWM

const int COLLECT_MOTOR_IN1 = 3; // PWM
const int COLLECT_MOTOR_IN2 = 4; // 数字脚（捡球电机只做开关，不用 PWM）

// 看门狗超时时间：超过该毫秒数没收到指令就停车
const unsigned long WATCHDOG_TIMEOUT_MS = 500;

String command = "";
unsigned long lastCmdTime = 0;

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);

  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  pinMode(COLLECT_MOTOR_IN1, OUTPUT);
  pinMode(COLLECT_MOTOR_IN2, OUTPUT);

  stopRobot();
  collectOff();
}

void loop() {
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    command.trim();

    // 收到指令，刷新看门狗计时
    lastCmdTime = millis();

    if (command.startsWith("SPEED ")) {
      // 格式：SPEED <left_pwm> <right_pwm>
      int spacePos = command.indexOf(' ', 6);
      if (spacePos > 0) {
        int leftPwm = command.substring(6, spacePos).toInt();
        int rightPwm = command.substring(spacePos + 1).toInt();

        leftPwm = constrain(leftPwm, 0, 255);
        rightPwm = constrain(rightPwm, 0, 255);

        drivePwm(leftPwm, rightPwm);
      }
    } else if (command == "MOVE_FORWARD") {
      moveForward();
    } else if (command == "MOVE_BACKWARD") {
      moveBackward();
    } else if (command == "TURN_LEFT") {
      turnLeft();
    } else if (command == "TURN_RIGHT") {
      turnRight();
    } else if (command == "STOP") {
      stopRobot();
    } else if (command == "COLLECT_ON") {
      collectOn();
    } else if (command == "COLLECT_OFF") {
      collectOff();
    } else if (command == "PING") {
      // 仅刷新看门狗计时，不改变任何动作
    }
  }

  // 串口看门狗：超时未收到指令，立即停车（含捡球电机）
  if (lastCmdTime != 0 && millis() - lastCmdTime > WATCHDOG_TIMEOUT_MS) {
    stopRobot();
    collectOff();
    lastCmdTime = 0;  // 置 0 避免每圈重复执行停车（上电后未收到过指令时看门狗不生效）
  }
}

// 差速 PWM 行驶：两轮都正转，速度不同实现转向
void drivePwm(int leftPwm, int rightPwm) {
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_IN2, 0);
  analogWrite(LEFT_MOTOR_IN1, leftPwm);

  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_IN1, rightPwm);
}

void moveForward() {
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);

  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

void moveBackward() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);

  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
}

void turnLeft() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);

  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

void turnRight() {
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);

  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
}

void stopRobot() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);

  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

void collectOn() {
  digitalWrite(COLLECT_MOTOR_IN1, HIGH);
  digitalWrite(COLLECT_MOTOR_IN2, LOW);
}

void collectOff() {
  digitalWrite(COLLECT_MOTOR_IN1, LOW);
  digitalWrite(COLLECT_MOTOR_IN2, LOW);
}
