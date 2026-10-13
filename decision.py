import math
import time

from config import (
    CENTER_TOLERANCE,
    KP,
    KD,
    MAX_SPEED,
    FORWARD_SPEED,
    DIST_PER_SQRT_AREA,
    COLLECT_DISTANCE,
    MISS_FRAME_LIMIT,
    USE_GROUND_PROJECTION,
    ANGLE_TOLERANCE,
    KP_ANGLE,
    KD_ANGLE,
    FORWARD_DISTANCE_GAIN,
    COLLECT_DISTANCE_M
)

from ground_projection import GroundProjection


class DecisionMaker:
    """
    方案 3（比例控制）+ 方案 4（连续丢帧判定）+ 方案 9（地面投影闭环）。

    两种模式（USE_GROUND_PROJECTION 切换）：
    - 像素模式：水平偏差像素 + 面积估距（方案 3 原逻辑）
    - 地面模式：真实方位角 bearing + 地面距离 distance（米）闭环，
      控制量与相机安装高度/分辨率解耦

    decide() 返回动作字典：
    - {"action": "COLLECT"}                     启动捡球
    - {"action": "DRIVE", "left": L, "right": R} 按左右轮速度行驶（PWM 0~255）
    - {"action": "SEARCH", "direction": ...}    丢目标，按方向搜索

    丢失判定逻辑：
    - detector 单帧漏检时返回外推/回放结果并标记 "missed": True，此时
      miss_count 递增，但继续按盲走方向行驶
    - 连续 MISS_FRAME_LIMIT 帧没有真正检测到目标，才判定真丢失进入 SEARCH
    - missed 结果不允许触发 COLLECT（防止对着旧位置捡球）
    """

    def __init__(self, frame_width, projection=None):
        self.frame_width = frame_width
        self.frame_center_x = frame_width // 2
        self.frame_half_width = frame_width / 2.0

        # 地面投影（USE_GROUND_PROJECTION=True 时必传）
        self.projection = projection
        self.use_ground = USE_GROUND_PROJECTION and projection is not None

        # 记录最后一次看到球时的转向方向，用于丢目标后继续朝该方向搜索
        self.last_seen_direction = "TURN_LEFT"

        # D 项状态：上一帧的偏差/方位角和时间
        self.prev_error = None
        self.prev_time = None

        # 连续丢失帧计数
        self.miss_count = 0

    def estimate_distance(self, area):
        """像素模式：用框面积反推相对距离：distance ≈ C / sqrt(area)"""
        if area <= 0:
            return float("inf")
        return DIST_PER_SQRT_AREA / math.sqrt(area)

    def decide(self, target):
        if target is None:
            # detector 没有任何历史（从未检测到）或已 reset
            self._reset_pd_state()
            return {"action": "SEARCH", "direction": self.last_seen_direction}

        # ---- 丢帧计数处理 ----
        missed = target.get("missed", False)

        if missed:
            self.miss_count += 1

            if self.miss_count >= MISS_FRAME_LIMIT:
                # 超过丢帧上限，判定真丢失
                self.miss_count = 0
                self._reset_pd_state()
                return {
                    "action": "SEARCH",
                    "direction": self.last_seen_direction
                }

            # 丢失但未超限：按最后已知方向盲走
            return {"action": "SEARCH", "direction": self.last_seen_direction}

        # 真正检测到目标，重置丢帧计数
        self.miss_count = 0

        if self.use_ground:
            return self._decide_ground(target)

        return self._decide_pixel(target)

    # ---- 地面投影模式（方案 9） ----

    def _decide_ground(self, target):
        bearing, distance = self.projection.bearing_and_distance(
            target["x"], target["w"], target["y"], target["h"]
        )

        # 更新最后看到的方向（用于丢目标后搜索）
        if bearing < -ANGLE_TOLERANCE:
            self.last_seen_direction = "TURN_LEFT"
        elif bearing > ANGLE_TOLERANCE:
            self.last_seen_direction = "TURN_RIGHT"

        # 距离足够近 -> 捡球
        if distance <= COLLECT_DISTANCE_M:
            return {"action": "COLLECT"}

        now = time.time()

        p_term = bearing  # 弧度，量级约 -1.5 ~ 1.5

        d_term = 0.0
        if (
            KD_ANGLE > 0
            and self.prev_error is not None
            and self.prev_time is not None
            and now > self.prev_time
        ):
            d_term = (bearing - self.prev_error) / (now - self.prev_time)

        self.prev_error = bearing
        self.prev_time = now

        # 差速：bearing 正值 = 目标在右侧 -> 右转
        turn = KP_ANGLE * p_term + KD_ANGLE * d_term
        turn = max(-1.0, min(1.0, turn))

        left = int(MAX_SPEED * (1.0 + turn))
        right = int(MAX_SPEED * (1.0 - turn))

        left = max(0, min(255, left))
        right = max(0, min(255, right))

        # 方位角已进入死区：直行，速度随距离衰减（近处减速防撞）
        if abs(bearing) <= ANGLE_TOLERANCE:
            fwd = int(min(FORWARD_SPEED, distance * FORWARD_DISTANCE_GAIN))
            fwd = max(fwd, 40)
            left = fwd
            right = fwd

        # 保证低速时电机仍能转动
        if 0 < left < 40:
            left = 40
        if 0 < right < 40:
            right = 40

        return {"action": "DRIVE", "left": left, "right": right}

    # ---- 像素模式（方案 3 原逻辑） ----

    def _decide_pixel(self, target):
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
            and self.prev_error is not None
            and self.prev_time is not None
            and now > self.prev_time
        ):
            d_term = (offset - self.prev_error) / (now - self.prev_time)
            d_term /= self.frame_half_width  # 与 P 项同尺度归一化

        self.prev_error = offset
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
        self.prev_error = None
        self.prev_time = None
