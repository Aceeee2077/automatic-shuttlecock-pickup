import cv2
import numpy as np


class GroundProjection:
    """
    方案 9：地面平面投影。

    用标定好的单应矩阵把图像坐标映射到以机器人为原点的地面坐标系（米）：
    - x 向右，y 向前，原点 = 机器人正下方地面投影点
    - bearing = atan2(x, y)：目标相对机器人正前方的方位角（弧度）
    - distance = sqrt(x^2 + y^2)：目标地面距离（米）

    PD 控制从此按真实方位角/距离闭环，不再受相机安装高度/分辨率影响。

    y 轴向前的含义：图像中越靠下的地面点离机器人越近。
    框底边中心是目标与地面的接触点，用它做投影。
    """

    def __init__(self, homography_path, calibration_path=None):
        try:
            data = np.load(homography_path)
        except FileNotFoundError:
            raise RuntimeError(
                f"{homography_path} not found. "
                "Run calibrate_homography.py first, "
                "or set USE_GROUND_PROJECTION = False in config.py."
            )

        self.H = data["H"].astype(np.float64)

        # 可选：去畸变（与标定脚本保持同一预处理）
        self.camera_matrix = None
        self.dist_coeffs = None

        if calibration_path is not None:
            try:
                calib = np.load(calibration_path)
                self.camera_matrix = calib["camera_matrix"]
                self.dist_coeffs = calib["dist_coeffs"]
            except FileNotFoundError:
                pass

    def undistort(self, frame):
        """相机帧去畸变（未标定时原样返回）。"""
        if self.camera_matrix is None:
            return frame

        return cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

    def project(self, px, py):
        """图像坐标 (px, py) -> 地面坐标 (x_m, y_m)。"""
        pt = np.array([[[float(px), float(py)]]], dtype=np.float64)
        mapped = cv2.perspectiveTransform(pt, self.H)
        return float(mapped[0, 0, 0]), float(mapped[0, 0, 1])

    def bearing_and_distance(self, box_x, box_w, box_y, box_h):
        """
        用框底边中心投影，返回 (bearing_rad, distance_m)。

        bearing: 正值 = 目标在右侧，负值 = 左侧，0 = 正前方
        distance: 地面距离（米）
        """
        cx = box_x + box_w / 2.0
        cy = box_y + box_h  # 底边中心

        gx, gy = self.project(cx, cy)

        # y <= 0 意味着目标在机器人身后或投影退化，方位角取 ±90 度
        if gy <= 1e-6:
            bearing = np.pi / 2.0 if gx >= 0 else -np.pi / 2.0
            return bearing, float(np.hypot(gx, gy))

        bearing = float(np.arctan2(gx, gy))

        return bearing, float(np.hypot(gx, gy))
