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
    COLLECT_DISTANCE_M,
    MULTI_BALL,
    NEXT_TARGET_MIN_SEP,
    MAX_ACCEL,
    USE_ACCEL_LIMIT
)

from ground_projection import GroundProjection


class DecisionMaker:
    """
    方案 3（比例控制）+ 方案 4（连续丢帧判定）+ 方案 9（地面投影闭环）
    + 方案 10（多球调度）+ 方案 11（捡球确认）+ 方案 12（加速度限幅）。

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

    捡球确认逻辑（方案 11）：
    - 进入 COLLECT 时记录被捡球中心，捡球期间 main 每帧调用
      observe_collecting()，统计"该位置附近仍看到球"的帧占比
    - finish_collecting() 在捡球结束时判定：占比超过
      COLLECT_CONFIRM_RATIO 判定失败（球没被捡走），返回 True 表示
      失败，main 就地重试；不超过则判定成功
    - 连续失败 MAX_COLLECT_RETRIES 次后进入冷却（COLLECT_RETRY_COOLDOWN
      秒），冷却期间 COLLECT 动作替换为 SEARCH，防止卡死在坏球上

    多球调度逻辑（方案 10）：
    - plan_next_target() 在捡球结束时从本帧候选框里挑下一个目标：
      排除被捡球位置附近的框（防止把同一颗球当两颗），选分数最高的
    - 找到则返回候选框字典，main 调用 detector.aim_at() 切换焦点，
      下一帧直接驶向新目标；找不到返回 None，退回正常 SEARCH
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

        # ---- 方案 11：捡球确认状态 ----
        # 进入 COLLECT 时被捡球中心
        self.collecting_center = None
        # 捡球期间总帧数 / 球仍可见帧数
        self.collecting_frames = 0
        self.collecting_visible_frames = 0

        # ---- 方案 11：失败重试与冷却 ----
        self.collect_retries = 0
        self.cooldown_until = 0.0

        # ---- 方案 10：被捡球中心（多球换目标时排除用） ----
        self.last_collected_center = None

        # ---- 方案 12：当前实际下发的轮速（用于加速度限幅） ----
        self.current_left = 0
        self.current_right = 0
        self.last_drive_time = None

    def estimate_distance(self, area):
        """像素模式：用框面积反推相对距离：distance ≈ C / sqrt(area)"""
        if area <= 0:
            return float("inf")
        return DIST_PER_SQRT_AREA / math.sqrt(area)

    def _limit_accel(self, left, right):
        """方案 12：轮速加速度限幅。

        限制相邻两次 DRIVE 之间每侧轮速的 PWM 增量（按帧间隔 dt 折算），
        平滑加减速，防止急转/急停打滑导致丢球。

        - USE_ACCEL_LIMIT=False 时直接透传
        - 重新进入 DRIVE（如捡球后重新出发）时从当前轮速平滑起步
        """
        if not USE_ACCEL_LIMIT:
            self.current_left = left
            self.current_right = right
            return left, right

        now = time.time()

        if self.last_drive_time is None or now <= self.last_drive_time:
            dt = 0.033
        else:
            dt = min(now - self.last_drive_time, 0.5)

        self.last_drive_time = now

        max_step = MAX_ACCEL * dt

        left = int(max(
            self.current_left - max_step,
            min(left, self.current_left + max_step)
        ))
        right = int(max(
            self.current_right - max_step,
            min(right, self.current_right + max_step)
        ))

        self.current_left = left
        self.current_right = right

        return left, right

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
            action = self._decide_ground(target)
        else:
            action = self._decide_pixel(target)

        # 方案 11：捡球冷却期间不触发 COLLECT，替换为 SEARCH
        if (
            action["action"] == "COLLECT"
            and time.time() < self.cooldown_until
        ):
            self._reset_pd_state()
            return {"action": "SEARCH", "direction": self.last_seen_direction}

        return action

    # ---- 方案 11：捡球确认 ----

    def start_collecting(self, target):
        """进入 COLLECT 时调用：记录被捡球中心，重置确认统计。"""
        self.collecting_center = (
            target["x"] + target["w"] / 2.0,
            target["y"] + target["h"]
        )
        self.collecting_frames = 0
        self.collecting_visible_frames = 0

    def observe_collecting(self, target):
        """捡球期间每帧调用：统计被捡球位置附近是否仍有球。

        detector 检测到目标且中心落在被捡球中心 COLLECT_CONFIRM_RADIUS
        内时，认为"球仍在原地"（即没被捡走）。
        """
        if self.collecting_center is None:
            return

        self.collecting_frames += 1

        if target is None or target.get("missed", False):
            return

        cx = target["x"] + target["w"] / 2.0
        cy = target["y"] + target["h"]

        dx = cx - self.collecting_center[0]
        dy = cy - self.collecting_center[1]

        if math.hypot(dx, dy) <= COLLECT_CONFIRM_RADIUS:
            self.collecting_visible_frames += 1

    def finish_collecting(self, candidates):
        """捡球结束时调用：判定是否成功，并尝试规划下一个目标。

        返回 (collect_failed, next_target)：
        - collect_failed: True 表示球没被捡走（可见占比超阈值），
          main 应就地重试
        - next_target: 方案 10 的下一个目标候选框，找不到为 None
        """
        collect_failed = False

        if COLLECT_CONFIRM and self.collecting_center is not None:
            ratio = (
                self.collecting_visible_frames / self.collecting_frames
                if self.collecting_frames > 0
                else 0.0
            )
            collect_failed = ratio >= COLLECT_CONFIRM_RATIO

        if collect_failed:
            self.collect_retries += 1

            if self.collect_retries > MAX_COLLECT_RETRIES:
                # 重试超限：进入冷却，放弃这颗球
                self.cooldown_until = time.time() + COLLECT_RETRY_COOLDOWN
                self.collect_retries = 0
        else:
            self.collect_retries = 0

        # 方案 10：成功捡到球（或确认关闭）时记录位置，供换目标排除用
        if not collect_failed:
            self.last_collected_center = self.collecting_center
            self.collecting_center = None
            self.collecting_frames = 0
            self.collecting_visible_frames = 0

        next_target = None

        if MULTI_BALL and not collect_failed:
            next_target = self._plan_next_target(candidates)

        return collect_failed, next_target

    def _plan_next_target(self, candidates):
        """从本帧候选框中挑选下一个目标（排除刚捡完球的位置附近）。"""
        if not candidates:
            return None

        center = self.last_collected_center

        for cand in candidates:
            if center is None:
                return cand

            cx = cand["x"] + cand["w"] / 2.0
            cy = cand["y"] + cand["h"]

            dx = cx - center[0]
            dy = cy - center[1]

            # 离被捡球位置足够远才是新球
            if math.hypot(dx, dy) >= NEXT_TARGET_MIN_SEP:
                return cand

        return None

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

        # 方案 12：加速度限幅后下发
        left, right = self._limit_accel(left, right)

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

        # 方案 12：加速度限幅后下发
        left, right = self._limit_accel(left, right)

        return {"action": "DRIVE", "left": left, "right": right}

    def _reset_pd_state(self):
        self.prev_error = None
        self.prev_time = None
