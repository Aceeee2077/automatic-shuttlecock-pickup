"""
方案 9（第 1 步）：相机内参标定脚本。

用法：
1. 打印一张 9x6 内角点的棋盘格标定板（OpenCV 官方 PDF 可直接打印）
2. 运行本脚本，从不同角度/距离拍 15~20 张棋盘格照片（按空格拍摄，Q 退出）
3. 标定结果保存到 calibration.npz，供主程序去畸变使用

之后在同一机位运行 calibrate_homography.py 完成地面单应标定。
"""

import glob

import cv2
import numpy as np

from config import CALIBRATION_PATH

# 棋盘格内角点数（列-1, 行-1）
CHESSBOARD = (9, 6)

# 标定板格子边长（米）。按实际打印尺寸修改！
SQUARE_SIZE = 0.025

# 已有照片目录（可选）：把图片放 calibration_imgs/ 下会自动读取
IMG_DIR = "calibration_imgs"


def collect_frames_from_camera():
    """从相机拍摄标定照片。"""
    cap = cv2.VideoCapture(0)
    frames = []

    if not cap.isOpened():
        return frames

    print("Press SPACE to capture, Q to finish (need >= 10 images).")

    while True:
        ret, frame = cap.read()

        if not ret:
            break

        display = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, CHESSBOARD, None
        )

        if found:
            cv2.drawChessboardCorners(display, CHESSBOARD, corners, found)
            cv2.putText(
                display, "DETECTED - SPACE to capture",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
            )
        else:
            cv2.putText(
                display, "no chessboard",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
            )

        cv2.putText(
            display, f"captured: {len(frames)}",
            (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2
        )

        cv2.imshow("Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        if key == ord(" ") and found:
            frames.append(frame.copy())
            print(f"captured {len(frames)}")

    cap.release()
    cv2.destroyWindow("Calibration")

    return frames


def load_frames_from_dir():
    """从目录读取已有标定照片。"""
    frames = []

    for path in (
        sorted(glob.glob(f"{IMG_DIR}/*.jpg"))
        + sorted(glob.glob(f"{IMG_DIR}/*.png"))
    ):
        frame = cv2.imread(path)

        if frame is not None:
            frames.append(frame)

    return frames


def calibrate(frames):
    obj_points = []
    img_points = []

    objp = np.zeros((CHESSBOARD[0] * CHESSBOARD[1], 3), dtype=np.float32)
    objp[:, :2] = (
        np.mgrid[0:CHESSBOARD[0], 0:CHESSBOARD[1]].T.reshape(-1, 2)
        * SQUARE_SIZE
    )

    used = 0

    for frame in frames:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, CHESSBOARD, None)

        if not found:
            continue

        corners = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            criteria=(
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30, 0.001
            )
        )

        obj_points.append(objp)
        img_points.append(corners)
        used += 1

    if used < 5:
        print(f"Not enough usable frames ({used}). Need >= 5.")
        return None

    h, w = frames[0].shape[:2]

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (w, h), None, None
    )

    print(f"\nUsed {used} frames, RMS reprojection error: {rms:.4f}")
    print("camera matrix:\n", camera_matrix)
    print("distortion coeffs:\n", dist_coeffs.ravel())

    np.savez(
        CALIBRATION_PATH,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        rms=rms
    )
    print(f"Saved to {CALIBRATION_PATH}")

    return camera_matrix, dist_coeffs


def main():
    frames = load_frames_from_dir()

    if len(frames) < 10:
        print(f"Only {len(frames)} images in {IMG_DIR}/, switching to camera.")
        frames = collect_frames_from_camera()

    if len(frames) < 5:
        print("Not enough frames, aborting.")
        return

    calibrate(frames)


if __name__ == "__main__":
    main()
