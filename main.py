import json
from pathlib import Path

import cv2
import time

import config as app_config
from preflight import print_report, validate_config

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
    CALIBRATION_PATH,
    USE_OBSTACLE_AVOID,
    USE_BATTERY_MONITOR,
    BATTERY_LOW_MV,
    BATTERY_CRITICAL_MV,
    USE_WEB_SERVER,
    WEB_SERVER_PORT,
    WEB_JPEG_QUALITY,
    STUCK_REVERSE_SECONDS,
    STUCK_REVERSE_SPEED,
    STUCK_TURN_SECONDS,
    STUCK_TURN_SPEED,
    USE_RUN_LOGGER,
    RUN_LOG_DIR,
    RUN_LOG_JPEG_QUALITY,
    RUN_LOG_EVERY_N_FRAMES,
    USE_DRIVE_CALIBRATION,
    DRIVE_CALIBRATION_PATH
)

from camera import Camera
from detector import ShuttleDetector
from decision import DecisionMaker
from robot_controller import RobotController
from ground_projection import GroundProjection


def load_drive_calibration():
    """读取直行标定系数；文件不存在或损坏时返回 (1.0, 1.0)。"""
    path = Path(DRIVE_CALIBRATION_PATH)

    if not path.is_file():
        return 1.0, 1.0

    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return (
            float(data.get("left_factor", 1.0)),
            float(data.get("right_factor", 1.0))
        )
    except (json.JSONDecodeError, ValueError) as e:
        print(f"Drive calibration file invalid ({e}), using 1.0/1.0.")
        return 1.0, 1.0


def _log_record(decision, target, miss_count, ground_info, obstacle,
                battery_mv, collected_count, obstacle_state):
    """组装一帧的日志记录（不含图像本体）。"""
    record = {
        "action": decision.get("action") if isinstance(decision, dict) else str(decision),
        "miss": miss_count,
        "collected": collected_count,
        "battery_mv": battery_mv,
        "obstacle_state": obstacle_state,
    }

    if isinstance(decision, dict):
        if decision["action"] == "DRIVE":
            record["left"] = decision["left"]
            record["right"] = decision["right"]
        if decision["action"] == "SEARCH":
            record["search_dir"] = decision.get("direction", "")

    if target is not None:
        record["target"] = {
            "x": int(target["x"]), "y": int(target["y"]),
            "w": int(target["w"]), "h": int(target["h"]),
            "conf": round(target.get("confidence", 0.0), 3),
            "id": target.get("id"),
            "missed": target.get("missed", False),
        }

    if obstacle is not None:
        record["obstacle"] = {
            "x": int(obstacle["x"]), "y": int(obstacle["y"]),
            "w": int(obstacle["w"]), "h": int(obstacle["h"]),
            "cls": obstacle.get("cls", "?"),
            "conf": round(obstacle.get("confidence", 0.0), 3),
        }

    if ground_info is not None:
        bearing, distance = ground_info
        record["bearing"] = round(bearing, 3)
        record["distance_m"] = round(distance, 3)

    return record


def draw_debug_info(
    frame, target, decision, miss_count=0, ground_info=None,
    obstacle=None, battery_mv=None, collected_count=0,
    obstacle_state="clear"
):
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

    # 方案 19：障碍物框红色显示
    if obstacle is not None:
        ox = obstacle["x"]
        oy = obstacle["y"]
        ow = obstacle["w"]
        oh = obstacle["h"]

        cv2.rectangle(display_frame, (ox, oy), (ox + ow, oy + oh), (0, 0, 255), 2)
        cv2.putText(
            display_frame,
            f"{obstacle.get('cls', '?')} {obstacle['confidence']:.2f}",
            (ox, max(oy - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
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

    # 第二行状态：捡球计数 / 电量 / 避障状态（方案 19 + 20）
    status_parts = [f"collected:{collected_count}"]

    if obstacle_state == "blocked":
        status_parts.append("OBSTACLE:WAIT")
    elif obstacle_state == "giveup":
        status_parts.append("OBSTACLE:GIVEUP")

    if battery_mv is not None:
        color_batt = (0, 255, 0)
        if battery_mv <= BATTERY_CRITICAL_MV:
            color_batt = (0, 0, 255)
        elif battery_mv <= BATTERY_LOW_MV:
            color_batt = (0, 255, 255)
        status_parts.append(f"batt:{battery_mv / 1000.0:.2f}V")

    status_text = "  ".join(status_parts)

    cv2.putText(
        display_frame,
        status_text,
        (20, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
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

    if action == "SETTLE":
        # 方案 26/27：到位观察或脱困冷却期间停车，仅发心跳防看门狗误停
        if last_command != "SETTLE":
            robot.stop()
        robot.heartbeat()
        return "SETTLE"

    return last_command


def _hold_with_heartbeat(robot, seconds):
    """方案 27：维持当前动作 seconds 秒，期间持续发心跳防看门狗误停。"""
    deadline = time.time() + seconds
    while time.time() < deadline:
        robot.heartbeat()
        time.sleep(0.2)


def execute_escape(robot, decision_maker):
    """方案 27：卡死脱困动作（时间驱动，一次性阻塞执行）。

    流程：后退 -> 原地转 -> 停止。脱困后 decision 端会清历史并
    重新驶向目标；脱困计数/基线由 decision 端管理。
    后退/转向期间持续发心跳，防止超过固件看门狗超时（500ms）被误停。
    """
    # 后退（先设速度再切方向，固件 PWM 沿用上次写入值）
    robot.set_speed(STUCK_REVERSE_SPEED, STUCK_REVERSE_SPEED)
    robot.backward()
    _hold_with_heartbeat(robot, STUCK_REVERSE_SECONDS)
    robot.stop()

    # 原地转向：单轮旋转（左停右进 -> 左转）
    robot.set_speed(0, STUCK_TURN_SPEED)
    _hold_with_heartbeat(robot, STUCK_TURN_SECONDS)
    robot.stop()

    # 脱困结束：通知 decision 端清卡死历史，重新驶向目标
    decision_maker.after_escape()
    print("Escape done (reverse + turn).")


def _web_status(decision, collected_count, battery_mv, obstacle,
                obstacle_state, decision_maker):
    """方案 22：构造推送给 Web 监控的状态字典。"""
    return {
        "mode": "ground" if decision_maker.use_ground else "pixel",
        "action": decision.get("action", "?"),
        "collected": collected_count,
        "battery_mv": battery_mv,
        "obstacle": (
            obstacle.get("cls") if obstacle else None
        ),
        "obstacle_state": obstacle_state,
        "miss_count": decision_maker.miss_count
    }


def main():
    camera = None
    robot = None
    run_logger = None

    # Validate config before opening the camera, model, serial port or any log
    # directory. A bad threshold or missing model should not leave hardware open.
    report = validate_config(
        app_config, base_dir=Path(__file__).resolve().parent
    )
    if not print_report(report):
        return

    try:
        try:
            camera = Camera()
        except RuntimeError as e:
            print(e)
            return

        # 方案 38：加载直行标定系数（无标定文件时系数为 1.0，等价于不修正）
        left_factor, right_factor = (1.0, 1.0)

        if USE_DRIVE_CALIBRATION:
            left_factor, right_factor = load_drive_calibration()

            if left_factor != 1.0 or right_factor != 1.0:
                print(
                    f"Drive calibration applied: "
                    f"L x{left_factor:.4f} R x{right_factor:.4f}"
                )

        robot = RobotController(
            use_serial=USE_SERIAL,
            port=SERIAL_PORT,
            baud_rate=BAUD_RATE,
            left_factor=left_factor,
            right_factor=right_factor
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

        mode = "ground projection" if decision_maker.use_ground else "pixel"

        # 方案 43：运行日志（可选），每帧 JPEG + JSON 落盘供离线回放
        if USE_RUN_LOGGER:
            from run_logger import RunLogger

            run_logger = RunLogger(
                root_dir=RUN_LOG_DIR,
                jpeg_quality=RUN_LOG_JPEG_QUALITY,
                every_n_frames=RUN_LOG_EVERY_N_FRAMES
            )
            run_logger.event("run_config", mode=mode)

        # 方案 22：Web 远程监控（可选）
        web_server = None

        if USE_WEB_SERVER:
            from web_server import WebMonitor

            web_server = WebMonitor(port=WEB_SERVER_PORT, jpeg_quality=WEB_JPEG_QUALITY)
            web_server.start()

        last_command = None
        collect_start_time = None
        is_collecting = False

        # 方案 20：捡球成功计数
        collected_count = 0

        # 方案 20：低电压报警节流（避免每帧刷屏）
        last_low_batt_warn = 0.0

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

            # 方案 19：障碍物检测（独立模型推理，不影响羽毛球检测）
            obstacle = detector.detect_obstacles(frame) if USE_OBSTACLE_AVOID else None

            # 方案 19：避障状态检查
            obstacle_state = decision_maker.check_obstacle(target, obstacle)

            # 方案 20：电池电压
            battery_mv = robot.get_battery_mv() if USE_BATTERY_MONITOR else None

            # 方案 20：低电压报警（每 5 秒最多一次），严重低电压强制停车
            if (
                battery_mv is not None
                and battery_mv <= BATTERY_CRITICAL_MV
            ):
                print(f"Battery critical: {battery_mv}mV, stopping.")
                robot.stop()
                robot.collect_off()
                last_command = "STOP"
                decision = {"action": "STOP"}

                # 严重低电压：持续停车，跳过正常决策（防止覆盖 STOP）
                display_frame = draw_debug_info(
                    frame, target, decision, decision_maker.miss_count,
                    None, obstacle, battery_mv, collected_count,
                    obstacle_state
                )

                if web_server is not None:
                    web_server.update(
                        display_frame,
                        _web_status(
                            decision, collected_count, battery_mv,
                            obstacle, obstacle_state, decision_maker
                        )
                    )

                # 方案 43：严重低电压停车期间也记帧
                if run_logger is not None:
                    run_logger.log_frame(
                        display_frame,
                        _log_record(
                            decision, target, decision_maker.miss_count,
                            None, obstacle, battery_mv,
                            collected_count, obstacle_state
                        )
                    )

                cv2.imshow("Robot Camera YOLO", display_frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                continue

            if (
                battery_mv is not None
                and battery_mv <= BATTERY_LOW_MV
            ):
                now = time.time()

                if now - last_low_batt_warn >= 5.0:
                    last_low_batt_warn = now
                    print(f"Battery low: {battery_mv}mV.")

                    if run_logger is not None:
                        run_logger.event("battery_low", mv=battery_mv)

            decision = decision_maker.decide(target)

            ground_info = None

            if decision_maker.use_ground and target is not None:
                ground_info = projection.bearing_and_distance(
                    target["x"], target["w"], target["y"], target["h"]
                )

            if is_collecting:
                # 捡球期间每帧观察：被捡球位置附近是否仍有球（方案 11）
                decision_maker.observe_collecting(target)

                if time.time() - collect_start_time >= COLLECT_TIME:
                    robot.collect_off()
                    robot.stop()

                    # 方案 11：视觉确认捡球是否成功
                    collect_failed, next_target = (
                        decision_maker.finish_collecting(
                            detector.last_candidates
                        )
                    )

                    is_collecting = False
                    collect_start_time = None

                    if collect_failed:
                        # 失败：就地重试（重新驶向同一位置触发 COLLECT）
                        print("Collect failed, retrying.")

                        if run_logger is not None:
                            run_logger.event("collect_failed")

                        last_command = None
                    elif next_target is not None:
                        # 方案 10：找到下一个目标，直接切换焦点驶向它
                        print("Next target found, switching.")

                        if run_logger is not None:
                            run_logger.event("collect_success",
                                             collected=collected_count + 1)

                        detector.aim_at(next_target)
                        collected_count += 1
                        last_command = None
                    else:
                        collected_count += 1

                        if run_logger is not None:
                            run_logger.event("collect_success",
                                             collected=collected_count)

                        last_command = "STOP"
                else:
                    decision = {"action": "COLLECTING"}
                    # 捡球期间不发行驶指令，但需要发心跳防止 Arduino 看门狗误停车
                    robot.heartbeat()

            else:
                # 方案 19：障碍物挡路处理
                if obstacle_state == "giveup":
                    # 持续遮挡超时：放弃该目标，进入搜索

                    if run_logger is not None:
                        run_logger.event("obstacle_giveup")

                    decision_maker._reset_pd_state()
                    detector.reset()
                    decision = {
                        "action": "SEARCH",
                        "direction": decision_maker.last_seen_direction
                    }
                    last_command = execute_decision(
                        robot, decision, last_command
                    )
                elif obstacle_state == "blocked":
                    # 障碍物挡路：停车等待（障碍物可能是移动的人）
                    if last_command != "OBSTACLE_WAIT":
                        robot.stop()
                        last_command = "OBSTACLE_WAIT"
                    decision = {"action": "OBSTACLE_WAIT"}
                elif decision["action"] == "COLLECT":
                    decision_maker.start_collecting(target)
                    robot.stop()
                    time.sleep(0.2)
                    robot.collect_on()
                    collect_start_time = time.time()
                    is_collecting = True
                    last_command = "COLLECT_ON"

                    if run_logger is not None:
                        run_logger.event("collect_start")
                elif decision["action"] == "ESCAPE":
                    # 方案 27：判定卡死，执行脱困（后退 + 原地转）
                    if run_logger is not None:
                        run_logger.event("escape")

                    execute_escape(robot, decision_maker)
                    # 脱困后重置 PD/指令去重，下一帧重新驶向目标
                    last_command = None
                    decision = {"action": "SETTLE"}
                else:
                    last_command = execute_decision(robot, decision, last_command)

            display_frame = draw_debug_info(
                frame, target, decision, decision_maker.miss_count,
                ground_info, obstacle, battery_mv, collected_count,
                obstacle_state
            )

            # 方案 43：每帧落盘（带调试标注的帧，回放时与现场屏幕一致）
            if run_logger is not None:
                run_logger.log_frame(
                    display_frame,
                    _log_record(
                        decision, target, decision_maker.miss_count,
                        ground_info, obstacle, battery_mv,
                        collected_count, obstacle_state
                    )
                )

            # 方案 22：推送到 Web 监控
            if web_server is not None:
                web_server.update(
                    display_frame,
                    _web_status(
                        decision, collected_count, battery_mv,
                        obstacle, obstacle_state, decision_maker
                    )
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

        if web_server is not None:
            web_server.stop()

        if run_logger is not None:
            run_logger.close()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
