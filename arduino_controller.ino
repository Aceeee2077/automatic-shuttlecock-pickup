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
*/

const int LEFT_MOTOR_IN1 = 5;
const int LEFT_MOTOR_IN2 = 6;

const int RIGHT_MOTOR_IN1 = 9;
const int RIGHT_MOTOR_IN2 = 10;

const int COLLECT_MOTOR_IN1 = 3;
const int COLLECT_MOTOR_IN2 = 4;

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

    if (command == "MOVE_FORWARD") {
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
