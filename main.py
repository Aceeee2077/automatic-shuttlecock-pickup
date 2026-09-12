import cv2
import time

from config import (
    USE_SERIAL,
    SERIAL_PORT,
    BAUD_RATE,
    FRAME_WIDTH,
    FRAME_HEIGHT,
    COLLECT_TIME,
    SHOW_MASK_WINDOW
)

from camera import Camera
from detector import ShuttleDetector
from decision import DecisionMaker
from robot_controller import RobotController


def draw_debug_info(frame, target, decision):
    display_frame = frame.copy()

    if target is not None:
        x = target["x"]
        y = target["y"]
        w = target["w"]
        h = target["h"]
        area = target["area"]

        confidence = target.get("confidence", None)

        cv2.rectangle(
            display_frame,
            (x, y),
            (x + w, y + h),
            (0, 255, 0),
            2
        )

        if confidence is not None:
            label = f"Shuttlecock {confidence:.2f}, area: {int(area)}"
        else:
            label = f"Shuttle area: {int(area)}"

        cv2.putText(
            display_frame,
            label,
            (x, max(y - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

    cv2.line(
        display_frame,
        (FRAME_WIDTH // 2, 0),
        (FRAME_WIDTH // 2, FRAME_HEIGHT),
        (255, 0, 0),
        2
    )

    cv2.putText(
        display_frame,
        f"Decision: {decision}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 255),
        2
    )

    return display_frame


def execute_decision(robot, decision, last_command):
    if decision == "SEARCH":
        if last_command != "TURN_LEFT":
            robot.turn_left()
            return "TURN_LEFT"

    elif decision == "TURN_LEFT":
        if last_command != "TURN_LEFT":
            robot.turn_left()
            return "TURN_LEFT"

    elif decision == "TURN_RIGHT":
        if last_command != "TURN_RIGHT":
            robot.turn_right()
            return "TURN_RIGHT"

    elif decision == "MOVE_FORWARD":
        if last_command != "MOVE_FORWARD":
            robot.forward()
            return "MOVE_FORWARD"

    return last_command


def main():
    try:
        camera = Camera()
    except RuntimeError as e:
        print(e)
        return

    robot = RobotController(
        use_serial=USE_SERIAL,
        port=SERIAL_PORT,
        baud_rate=BAUD_RATE
    )

    detector = ShuttleDetector()
    decision_maker = DecisionMaker(FRAME_WIDTH)

    last_command = None
    collect_start_time = None
    is_collecting = False

    print("Program started with YOLO detector.")
    print("Press Q to quit.")

    while True:
        frame = camera.read()

        if frame is None:
            print("Cannot read frame.")
            break

        target, mask = detector.detect(frame)

        decision = decision_maker.decide(target)

        if is_collecting:
            if time.time() - collect_start_time >= COLLECT_TIME:
                robot.collect_off()
                robot.stop()
                is_collecting = False
                collect_start_time = None
                last_command = "STOP"
            else:
                decision = "COLLECTING"

        else:
            if decision == "COLLECT":
                robot.stop()
                time.sleep(0.2)
                robot.collect_on()
                collect_start_time = time.time()
                is_collecting = True
                last_command = "COLLECT_ON"
            else:
                last_command = execute_decision(robot, decision, last_command)

        display_frame = draw_debug_info(frame, target, decision)

        cv2.imshow("Robot Camera YOLO", display_frame)

        if SHOW_MASK_WINDOW and mask is not None:
            cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

    robot.stop()
    robot.collect_off()
    robot.close()
    camera.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()