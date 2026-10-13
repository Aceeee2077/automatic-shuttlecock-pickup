"""
方案 9（第 2 步）：地面单应矩阵标定脚本。

前置：先运行 camera_calibration.py 生成内参（可选但推荐）。

用法：
1. 把小车（相机）摆到比赛时的实际机位，保持不动
2. 在相机正前方地面上放一个参照物（如羽毛球），用尺子量出它到
   相机正下方投影点的水平距离 D_REF（米）
3. 运行本脚本，画面里用鼠标依次点击 4 个地面参考点：
   - 点 A：相机正下方地面投影点（机器人自身位置，距离 0）
   - 点 B：正前方距离 D_REF 处的地面点
   - 点 C/D：与 A/B 等距的左右两个地面点（构成已知宽度的正方形）
4. 输入 A-B 的实际距离和 A-C/A-D 的实际宽度，保存 homography.npz

原理：单应矩阵把图像坐标映射到以机器人为原点的地面坐标系（米），
用已知长度的正方形四角即可求出。

点击顺序（逆时针或顺时针均可，但需与 WORLD_POINTS 顺序一致）：
  0: (0, 0)          机器人正下方
  1: (0, D_REF)      正前方 D_REF 米
  2: (W, D_REF)      右前方
  3: (W, 0)          右侧
"""

import cv2
import numpy as np

from config import HOMOGRAPHY_PATH, CALIBRATION_PATH, FRAME_WIDTH, FRAME_HEIGHT

# 地面坐标系（米）：x 向右，y 向前，原点在机器人正下方
# 点击顺序对应下面四个世界点（W 和 D_REF 运行时输入）
WORLD_TEMPLATE = [
    (0.0, 0.0),
    (0.0, None),   # y = D_REF
    (None, None),  # (W, D_REF)
    (None, 0.0),   # (W, 0)
]

clicked_points = []
display_img = None


def on_mouse(event, x, y, flags, param):
    global display_img

    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
        clicked_points.append((x, y))
        cv2.circle(display_img, (x, y), 6, (0, 0, 255), -1)
        cv2.putText(
            display_img, f"{len(clicked_points) - 1}",
            (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
        )


def main():
    global display_img

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Cannot open camera.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("Cannot read frame.")
        return

    # 可选：去畸变后再标定（与主程序流程一致）
    try:
        calib = np.load(CALIBRATION_PATH)
        camera_matrix = calib["camera_matrix"]
        dist_coeffs = calib["dist_coeffs"]
        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
        print("Undistorted with calibration.npz")
    except (FileNotFoundError, KeyError):
        print("No calibration.npz, using raw frame (lens distortion may skew results).")

    display_img = frame.copy()

    print("Click 4 ground points in order:")
    print("  0: robot bottom projection (0, 0)")
    print("  1: front point at D_REF meters (0, D_REF)")
    print("  2: right-front (W, D_REF)")
    print("  3: right side (W, 0)")
    print("Press R to reset clicks, Q/ESC to finish.")

    cv2.namedWindow("Homography Calibration", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Homography Calibration", on_mouse)

    while True:
        cv2.imshow("Homography Calibration", display_img)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("r"):
            clicked_points.clear()
            display_img = frame.copy()
            print("Reset.")

        if key in (ord("q"), 27):
            break

    cv2.destroyWindow("Homography Calibration")

    if len(clicked_points) != 4:
        print(f"Need exactly 4 points, got {len(clicked_points)}. Aborting.")
        return

    d_ref = float(input("D_REF (m, robot->front point distance): "))
    width = float(input("W (m, square width between front and right points): "))

    world_points = [
        (0.0, 0.0),
        (0.0, d_ref),
        (width, d_ref),
        (width, 0.0),
    ]

    src = np.array(clicked_points, dtype=np.float64)
    dst = np.array(world_points, dtype=np.float64)

    H, status = cv2.findHomography(src, dst)

    if H is None:
        print("findHomography failed (points may be degenerate).")
        return

    # 用对角线长度验证精度
    diag_world = np.linalg.norm(dst[0] - dst[2])

    repro = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
    err = np.linalg.norm(repro - dst, axis=1)

    diag_repro = np.linalg.norm(repro[0] - repro[2])

    print(f"\nHomography:\n{H}")
    print(
        f"Validation: diagonal projects to {diag_repro:.3f}m "
        f"(expected {diag_world:.3f}m)"
    )
    print(f"Per-point reprojection error (m): {err.round(4)}")

    np.savez(HOMOGRAPHY_PATH, H=H, points_img=src, points_world=dst)
    print(f"Saved to {HOMOGRAPHY_PATH}")


if __name__ == "__main__":
    main()
