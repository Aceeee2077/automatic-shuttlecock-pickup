import math
import time

from config import (
    CENTER_TOLERANCE,
    KP,
    KD,
    MAX_SPEED,
    FORWARD_SPEED,
    DIST_PER_SQRT_AREA,
    COLLECT_DISTANCE
)


class DecisionMaker:
    """
    比例控制版本（方案 3）。

    decide() 返回动作字典而不是单个字符串：
    - {"action": "COLLECT"}                     启动捡球
    - {"action": "DRIVE", "left": L, "right": R} 按左右轮速度行驶（PWM 0~255）
    - {"action": "SEARCH", "direction": ...}    丢目标，按方向搜索
    """

    def __init__(self, frame_width):
        self.frame_width = frame_width
        self.frame_center_x = frame_width // 2
        self.frame_half_width = frame_width / 2.0

        # 记录最后一次看到球时的转向方向，用于丢目标后继续朝该方向搜索
        self.last_seen_direction = "TURN_LEFT"

        # D 项状态：上一帧的水平偏差和时间
        self.prev_offset = None
        self.prev_time = None

    def estimate_distance(self, area):
        """用框面积反推相对距离：distance ≈ C / sqrt(area)"""
        if area <= 0:
            return float("inf")
        return DIST_PER_SQRT_AREA / math.sqrt(area)

    def decide(self, target):
        if target is None:
            self._reset_pd_state()
            return {"action": "SEARCH", "direction": self.last_seen_direction}

        target_center_x = target["x"] + target["w"] // 2
        target_area = target["area"]

        offset = target_center_x - self.frame_center_x

        # 更新最后看到的方向（用于丢目标后搜索）
        if offset < -CENTER_TOLERANCE:
            self.last_seen_direction = "TURN_LEFT"
        elif offset > CENTER_TOLERANCE:
            self.last_seen_direction = "TURN_RIGHT"

        # 距离足够近 -> 捡球（代替固定面积阈值，换分辨率/相机不用重新调参）
        distance = self.estimate_distance(target_area)
        if distance <= COLLECT_DISTANCE:
            return {"action": "COLLECT"}

        # ---- PD 控制计算差速 ----
        now = time.time()

        p_term = offset / self.frame_half_width  # 归一化偏差 -1 ~ 1

        d_term = 0.0
        if (
            KD > 0
            and self.prev_offset is not None
            and self.prev_time is not None
            and now > self.prev_time
        ):
            d_term = (offset - self.prev_offset) / (now - self.prev_time)
            d_term /= self.frame_half_width  # 与 P 项同尺度归一化

        self.prev_offset = offset
        self.prev_time = now

        # 差速：正值表示目标在右侧 -> 右轮慢、左轮快（右转）
        turn = KP * p_term + KD * d_term
        turn = max(-1.0, min(1.0, turn))

        left = int(MAX_SPEED * (1.0 + turn))
        right = int(MAX_SPEED * (1.0 - turn))

        # 限幅，避免倒车：原地转时一侧为 0，另一侧为 2*MAX_SPEED
        left = max(0, min(255, left))
        right = max(0, min(255, right))

        # 目标已在死区内：直行前进
        if abs(offset) <= CENTER_TOLERANCE:
            left = FORWARD_SPEED
            right = FORWARD_SPEED

        # 保证低速时电机仍能转动
        if 0 < left < 40:
            left = 40
        if 0 < right < 40:
            right = 40

        return {"action": "DRIVE", "left": left, "right": right}

    def _reset_pd_state(self):
        self.prev_offset = None
        self.prev_time = None
