import cv2
import time

from config import (
    USE_SERIAL,
    SERIAL_PORT,
    BAUD_RATE,
    FRAME_WIDTH,
    FRAME_HEIGHT,
    COLLECT_TIME,
    SHOW_MASK_WINDOW,
    USE_GROUND_PROJECTION,
    HOMOGRAPHY_PATH,
    CALIBRATION_PATH
)

from camera import Camera
from detector import ShuttleDetector
from decision import DecisionMaker
from robot_controller import RobotController
from ground_projection import GroundProjection


def draw_debug_info(frame, target, decision, miss_count=0, ground_info=None):
    display_frame = frame.copy()

    if target is not None:
        x = target["x"]
        y = target["y"]
        w = target["w"]
        h = target["h"]
        area = target["area"]

        confidence = target.get("confidence", None)
        track_id = target.get("id", None)
        missed = target.get("missed", False)
        predicted = target.get("predicted", False)

        # 丢失回放的旧框用黄色画，正常检测用绿色
        color = (0, 255, 255) if missed else (0, 255, 0)

        cv2.rectangle(
            display_frame,
            (x, y),
            (x + w, y + h),
            color,
            2
        )

        id_text = f", id: {track_id}" if track_id is not None else ""
        if confidence is not None:
            label = f"Shuttlecock {confidence:.2f}{id_text}, area: {int(area)}"
        else:
            label = f"Shuttle{id_text} area: {int(area)}"

        if missed:
            label += " (STALE)"

        if predicted:
            label += " (PRED)"

        cv2.putText(
            display_frame,
            label,
            (x, max(y - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2
        )

        # 地面模式：画投影点和地面距离
        if ground_info is not None:
            bearing, distance = ground_info
            cx = int(x + w / 2.0)
            cy = int(y + h)

            cv2.circle(display_frame, (cx, cy), 5, (255, 0, 255), -1)
            cv2.putText(
                display_frame,
                f"{distance:.2f}m, {bearing * 57.2958:.0f}deg",
                (cx + 8, cy + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 0, 255),
                2
            )

    cv2.line(
        display_frame,
        (FRAME_WIDTH // 2, 0),
        (FRAME_WIDTH // 2, FRAME_HEIGHT),
        (255, 0, 0),
        2
    )

    # decision 现在是动作字典，显示形式为 "action | L speed R speed"
    if isinstance(decision, dict):
        if decision["action"] == "DRIVE":
            text = f"DRIVE L:{decision['left']} R:{decision['right']}"
        else:
            text = decision["action"]
            if decision["action"] == "SEARCH":
                text += f" {decision.get('direction', '')}"
    else:
        text = str(decision)

    # 显示丢帧计数，方便调试跟踪稳定性
    text += f"  miss:{miss_count}"

    cv2.putText(
        display_frame,
        text,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 255),
        2
    )

    return display_frame


def execute_decision(robot, decision, last_command):
    """
    执行动作字典（方案 3 比例控制）。

    DRIVE 是连续控制，每帧都发送 SPEED 指令（速度一直在变，不需要去重）；
    其它离散动作仍按 last_command 去重，避免重复发送。
    返回值统一为 last_command 的字符串，供离散动作去重用。
    """
    action = decision["action"]

    if action == "DRIVE":
        robot.set_speed(decision["left"], decision["right"])
        return "DRIVE"

    if action == "SEARCH":
        direction = decision["direction"]
        if last_command != direction:
            if direction == "TURN_LEFT":
                robot.turn_left()
            else:
                robot.turn_right()
            return direction

    return last_command


def main():
    camera = None
    robot = None

    try:
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

        projection = None

        if USE_GROUND_PROJECTION:
            try:
                projection = GroundProjection(
                    HOMOGRAPHY_PATH,
                    calibration_path=CALIBRATION_PATH
                )
            except RuntimeError as e:
                print(e)
                return

        decision_maker = DecisionMaker(FRAME_WIDTH, projection=projection)

        last_command = None
        collect_start_time = None
        is_collecting = False

        mode = "ground projection" if decision_maker.use_ground else "pixel"
        print(f"Program started with YOLO detector ({mode} mode).")
        print("Press Q to quit.")

        while True:
            frame = camera.read()

            if frame is None:
                print("Cannot read frame.")
                break

            # 地面模式：先去畸变再检测（与标定流程一致）
            if projection is not None:
                frame = projection.undistort(frame)

            target, mask = detector.detect(frame)

            decision = decision_maker.decide(target)

            ground_info = None

            if decision_maker.use_ground and target is not None:
                ground_info = projection.bearing_and_distance(
                    target["x"], target["w"], target["y"], target["h"]
                )

            if is_collecting:
                if time.time() - collect_start_time >= COLLECT_TIME:
                    robot.collect_off()
                    robot.stop()
                    is_collecting = False
                    collect_start_time = None
                    last_command = "STOP"
                else:
                    decision = {"action": "COLLECTING"}
                    # 捡球期间不发行驶指令，但需要发心跳防止 Arduino 看门狗误停车
                    robot.heartbeat()

            else:
                if decision["action"] == "COLLECT":
                    robot.stop()
                    time.sleep(0.2)
                    robot.collect_on()
                    collect_start_time = time.time()
                    is_collecting = True
                    last_command = "COLLECT_ON"
                else:
                    last_command = execute_decision(robot, decision, last_command)

            display_frame = draw_debug_info(
                frame, target, decision, decision_maker.miss_count,
                ground_info
            )

            cv2.imshow("Robot Camera YOLO", display_frame)

            if SHOW_MASK_WINDOW and mask is not None:
                cv2.imshow("Mask", mask)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

    except KeyboardInterrupt:
        print("\nStopped by user (Ctrl+C).")
    finally:
        # 无论正常退出还是 Ctrl+C 中断，都确保机器人停止、资源释放
        if robot is not None:
            robot.stop()
            robot.collect_off()
            robot.close()

        if camera is not None:
            camera.release()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
