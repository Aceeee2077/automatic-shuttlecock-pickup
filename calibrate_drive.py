"""
直行漂移标定脚本。

两轮底盘的左右电机/电池内阻不一致，直行会缓慢偏航，控制层要一直"擦"
这个偏差。本脚本测量直行时的偏航量，算出左右轮速度修正系数，存到
drive_calibration.json，RobotController 下发速度时自动应用。

原理：
    前方固定参照物（锥桶/立杆）与车同一直线 -> 直行 N 秒 -> 用模板匹配
    跟踪参照物底部的像素位移 dx 和图像行位移 dy。由 dy 与用户输入的
    初始距离反推出实际行驶距离，再由 dx 换算横向偏移和偏航角，
    得出左右轮真实速度比误差，生成修正系数。

用法：
    python calibrate_drive.py                          # 默认参数
    python calibrate_drive.py --distance 4 --track 0.35 --duration 3 --speed 100

操作：
    1. 把一个竖直参照物（锥桶、立杆）放在车正前方几米，确保直行会撞上它
    2. 运行脚本后按回车确认距离/轮距参数，然后在画面上点击参照物与地面
       的交点（底部）
    3. 按 C 开始直行测量，结束后自动保存 drive_calibration.json
    Q 取消退出
"""

import argparse
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2

from config import CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT, DRIVE_CALIBRATION_PATH


def open_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        print(f"Cannot open camera (index {CAMERA_INDEX}).")
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    time.sleep(1.0)
    return cap


def take_reference(cap):
    """让用户点击参照物底部，返回 (frame, x0, y0)。"""
    print("在画面上点击参照物与地面的交点（底部），按 C 确认，Q 取消。")
    state = {"pos": None}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            state["pos"] = (x, y)

    win = "Calibrate Drive"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse)

    while True:
        ok, frame = cap.read()

        if not ok:
            print("Cannot read frame.")
            return None

        if state["pos"] is not None:
            x, y = state["pos"]
            cv2.circle(frame, (x, y), 8, (0, 0, 255), 2)
            cv2.line(frame, (0, y), (FRAME_WIDTH, y), (0, 0, 255), 1)
            cv2.line(frame, (x, 0), (x, FRAME_HEIGHT), (0, 0, 255), 1)

        cv2.putText(
            frame, "Click the base of the marker (where it meets the ground)",
            (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
        )
        cv2.imshow(win, frame)

        key = cv2.waitKey(30) & 0xFF

        if key == ord("q"):
            cv2.destroyAllWindows()
            return None

        if key == ord("c") and state["pos"] is not None:
            cv2.destroyAllWindows()
            return frame, state["pos"][0], state["pos"][1]


def track_marker(frame, template, search_x, search_y):
    """在 (search_x, search_y) 附近用模板匹配定位参照物底部。

    返回 (x, y, score)；匹配质量差返回 None。
    """
    th, tw = template.shape[:2]
    x_lo = max(0, search_x - 150)
    x_hi = min(FRAME_WIDTH - tw, search_x + 150)
    y_lo = max(0, search_y - 80)
    y_hi = min(FRAME_HEIGHT - th, search_y + 80)

    if x_hi <= x_lo or y_hi <= y_lo:
        return None

    roi = frame[y_lo:y_hi, x_lo:x_hi]

    if roi.shape[0] < th or roi.shape[1] < tw:
        return None

    result = cv2.matchTemplate(roi, template, cv2.TM_CCOEFF_NORMED)
    _, max_val, _, max_loc = cv2.minMaxLoc(result)

    # 匹配分过低认为丢失（光照/遮挡）
    if max_val < 0.6:
        return None

    return x_lo + max_loc[0] + tw // 2, y_lo + max_loc[1] + th // 2, max_val


def main():
    parser = argparse.ArgumentParser(description="Straight-line drift calibration")
    parser.add_argument("--distance", type=float, default=5.0,
                        help="参照物初始距离（米），默认 5.0")
    parser.add_argument("--track", type=float, default=0.30,
                        help="轮距（米，左右轮轴间距），默认 0.30")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="直行时长（秒），默认 3.0")
    parser.add_argument("--speed", type=int, default=100,
                        help="直行 PWM 速度 0~255，默认 100")
    parser.add_argument("--fov", type=float, default=60.0,
                        help="相机水平视场角（度），默认 60")
    parser.add_argument("--output", default=DRIVE_CALIBRATION_PATH,
                        help="标定结果输出路径")
    args = parser.parse_args()

    cap = open_camera()

    if cap is None:
        return 1

    print(f"参照物距离: {args.distance} m   轮距: {args.track} m")
    print(f"直行 {args.duration} 秒 @ PWM {args.speed}")
    input("参数无误后按回车继续（参照物必须放在正前方直行路径上）...")

    ref = take_reference(cap)

    if ref is None:
        print("Cancelled.")
        cap.release()
        return 1

    frame0, x0, y0 = ref

    # 以点击点为中心的模板（底部纹理：参照物下段 + 接触面）
    tw, th = 120, 120
    tx0 = max(0, min(FRAME_WIDTH - tw, x0 - tw // 2))
    ty0 = max(0, min(FRAME_HEIGHT - th, y0 - th // 2))
    template = frame0[ty0:ty0 + th, tx0:tx0 + tw].copy()

    # 直行：SPEED 指令直发串口不可用（本脚本独立运行），
    # 若已连 Arduino 请保持 main 的串口占用方式；此处通过 RobotController 下发
    from robot_controller import RobotController
    from config import USE_SERIAL, SERIAL_PORT, BAUD_RATE

    robot = RobotController(
        use_serial=USE_SERIAL, port=SERIAL_PORT, baud_rate=BAUD_RATE
    )

    print("1 秒后开始直行，注意不要碰倒参照物...")
    time.sleep(1.0)
    robot.set_speed(args.speed, args.speed)

    t_start = time.time()
    samples = []
    last_pos = (x0, y0)
    win = "Calibrating"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    print("直行中，Q 取消（会作废本次标定）...")

    try:
        while True:
            ok, frame = cap.read()

            if not ok:
                break

            elapsed = time.time() - t_start

            if elapsed >= args.duration:
                break

            found = track_marker(frame, template, *last_pos)

            if found is None:
                print("WARNING: marker lost, retrying...")
            else:
                last_pos = (found[0], found[1])
                samples.append((elapsed, found[0], found[1], found[2]))
                cv2.circle(frame, (found[0], found[1]), 6, (0, 0, 255), -1)

            cv2.line(frame, (x0, y0), (last_pos[0], last_pos[1]), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"t={elapsed:.1f}s  dx={last_pos[0] - x0:+d}px  dy={last_pos[1] - y0:+d}px",
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
            )
            cv2.imshow(win, frame)

            if cv2.waitKey(30) & 0xFF == ord("q"):
                print("Cancelled during drive.")
                robot.stop()
                cv2.destroyAllWindows()
                cap.release()
                robot.close()
                return 1

    finally:
        robot.stop()

    cv2.destroyAllWindows()

    if len(samples) < 5:
        print("Not enough valid samples, calibration aborted.")
        cap.release()
        robot.close()
        return 1

    # 取最后一条有效样本作为终点
    _, x1, y1, _ = samples[-1]
    dx = x1 - x0
    dy = y1 - y0

    cy = FRAME_HEIGHT // 2

    # 由 y 方向投影反推距离：d = K / (y - cy)，K = d0 * (y0 - cy)
    if y0 <= cy or y1 <= cy:
        print("参照物底部必须位于画面中线以下（相机需向下俯看）。")
        cap.release()
        robot.close()
        return 1

    K = args.distance * (y0 - cy)
    d1 = K / (y1 - cy)
    d_driven = args.distance - d1

    if d_driven <= 0.1:
        print(f"行驶距离估计异常（{d_driven:.2f} m），请放慢速度或拉远参照物。")
        cap.release()
        robot.close()
        return 1

    # 横向偏移（米）：dx 像素按参照物初始距离处的米/像素比例换算
    px_per_m = FRAME_WIDTH / (2.0 * args.distance * math.tan(math.radians(args.fov / 2.0)))
    lateral = dx / px_per_m

    # 偏航角（小角度）：横向偏移 ≈ theta * d / 2
    theta = 2.0 * lateral / d_driven

    # 左右轮速度比误差：(Vr - Vl) / V_avg = theta * 轮距 / 行驶距离
    e = theta * args.track / d_driven

    # 修正系数：左轮 ×(1+e/2)，右轮 ×(1-e/2)，使真实轮速相等
    left_factor = 1.0 + e / 2.0
    right_factor = 1.0 - e / 2.0

    if abs(e) > 0.15:
        print(f"WARNING: 轮速差达 {e * 100:.1f}%，偏大，请检查机械是否卡滞。")

    result = {
        "left_factor": round(left_factor, 4),
        "right_factor": round(right_factor, 4),
        "drift_pixels": dx,
        "distance_driven_m": round(d_driven, 3),
        "heading_error_rad": round(theta, 4),
        "wheel_ratio_error": round(e, 4),
        "marker_distance_m": args.distance,
        "track_m": args.track,
        "created": datetime.now().isoformat(timespec="seconds"),
    }

    out_path = Path(args.output)
    out_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    print("\n===== 标定结果 =====")
    print(f"行驶距离     : {d_driven:.2f} m")
    print(f"横向偏移     : {lateral * 100:.1f} cm ({dx:+d} px)")
    print(f"偏航角       : {theta * 57.2958:.2f} deg")
    print(f"轮速比误差   : {e * 100:.2f}% ({'右轮偏快' if e > 0 else '左轮偏快'})")
    print(f"左轮修正系数 : {left_factor:.4f}")
    print(f"右轮修正系数 : {right_factor:.4f}")
    print(f"\n已保存: {out_path}")
    print("RobotController 在 USE_DRIVE_CALIBRATION=True 时自动应用。")

    cap.release()
    robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
