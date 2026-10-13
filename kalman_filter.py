import numpy as np


class ConstantVelocityKalman:
    """
    方案 8：匀速模型 Kalman 滤波，平滑目标框底边中心 (cx, cy)。

    相比 EMA：
    - 漏检期间可以 predict() 外推目标运动轨迹，盲走更聪明
    - 匀速模型对匀速移动目标零滞后，EMA 恒滞后

    状态向量 x = [cx, cy, vx, vy]，测量 z = [cx, cy]。
    dt 由每次 update 时传入，帧率波动不影响模型。
    """

    def __init__(self, pos_noise, vel_noise, meas_noise):
        # 过程噪声（对角）：位置与速度
        self.q_pos = pos_noise
        self.q_vel = vel_noise
        # 测量噪声（像素方差）
        self.r_meas = meas_noise

        # 状态 [cx, cy, vx, vy]
        self.x = np.zeros(4, dtype=np.float64)
        self.P = np.eye(4, dtype=np.float64) * 100.0

        self.initialized = False

    def _F(self, dt):
        F = np.eye(4, dtype=np.float64)
        F[0, 2] = dt
        F[1, 3] = dt
        return F

    def _Q(self, dt):
        dt2 = dt * dt
        return np.diag([
            self.q_pos * dt2,
            self.q_pos * dt2,
            self.q_vel * dt,
            self.q_vel * dt
        ])

    def predict(self, dt):
        """时间更新。dt 是距上次更新的秒数。"""
        if not self.initialized:
            return None

        F = self._F(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self._Q(dt)

        return self.x[0], self.x[1]

    def update(self, cx, cy, dt):
        """测量更新。dt 是距上次测量的秒数。"""
        if not self.initialized:
            self.x[:] = (cx, cy, 0.0, 0.0)
            self.P = np.eye(4, dtype=np.float64) * 100.0
            self.initialized = True
            return self.x[0], self.x[1]

        # 预测
        F = self._F(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self._Q(dt)

        # 更新（测量矩阵 H 只取位置）
        H = np.array([[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0]], dtype=np.float64)
        R = np.eye(2, dtype=np.float64) * self.r_meas

        y = np.array([cx, cy]) - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I_KH = np.eye(4, dtype=np.float64) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        return self.x[0], self.x[1]

    def get_velocity(self):
        """返回当前估计的像素速度 (vx, vy)。"""
        return self.x[2], self.x[3]
