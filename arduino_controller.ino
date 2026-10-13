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
*/

const int LEFT_MOTOR_IN1 = 5;   // PWM
const int LEFT_MOTOR_IN2 = 6;   // PWM

const int RIGHT_MOTOR_IN1 = 9;  // PWM
const int RIGHT_MOTOR_IN2 = 10; // PWM

const int COLLECT_MOTOR_IN1 = 3; // PWM
const int COLLECT_MOTOR_IN2 = 4; // 数字脚（捡球电机只做开关，不用 PWM）

String command = "";

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
    }
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
