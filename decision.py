import math
import time
from collections import deque

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
    USE_ACCEL_LIMIT,
    USE_OBSTACLE_AVOID,
    OBSTACLE_STOP_DISTANCE_M,
    OBSTACLE_AREA_NEAR,
    OBSTACLE_BLOCK_TIMEOUT,
    COLLECT_CONFIRM,
    COLLECT_CONFIRM_RATIO,
    COLLECT_CONFIRM_RADIUS,
    MAX_COLLECT_RETRIES,
    COLLECT_RETRY_COOLDOWN,
    USE_STAGED_APPROACH,
    STAGED_NEAR_DISTANCE_M,
    STAGED_NEAR_SPEED,
    STAGED_KP_ANGLE,
    STAGED_KD_ANGLE,
    STAGED_SETTLE_TIME,
    USE_STUCK_DETECT,
    STUCK_DURATION,
    STUCK_DISTANCE_EPS,
    STUCK_MAX_ESCAPES,
    STUCK_RETRY_COOLDOWN
)

from ground_projection import GroundProjection


class DecisionMaker:
    """
    方案 3（比例控制）+ 方案 4（连续丢帧判定）+ 方案 9（地面投影闭环）
    + 方案 10（多球调度）+ 方案 11（捡球确认）+ 方案 12（加速度限幅）
    + 方案 19（障碍物避障）+ 方案 26（三段式接近）+ 方案 27（卡死脱困）。

    两种模式（USE_GROUND_PROJECTION 切换）：
    - 像素模式：水平偏差像素 + 面积估距（方案 3 原逻辑）
    - 地面模式：真实方位角 bearing + 地面距离 distance（米）闭环，
      控制量与相机安装高度/分辨率解耦

    decide() 返回动作字典：
    - {"action": "COLLECT"}                     启动捡球
    - {"action": "SETTLE"}                      到位暂停观察（方案 26）
    - {"action": "DRIVE", "left": L, "right": R} 按左右轮速度行驶（PWM 0~255）
    - {"action": "ESCAPE"}                      判定卡死，请 main 执行脱困（方案 27）
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

        # ---- 方案 19：障碍物遮挡状态 ----
        # 障碍物首次挡住目标的时刻（None 表示未遮挡）
        self.blocked_since = None

        # ---- 方案 26：三段式接近状态 ----
        # 到位暂停观察开始时刻（None 表示未在观察）
        self.settle_since = None

        # ---- 方案 27：卡死检测与脱困状态（仅地面模式生效） ----
        # 行驶中的 (time, distance) 历史，仅 DRIVE 期间记录
        self.stuck_history = deque(maxlen=600)
        # 当前目标的脱困次数
        self.stuck_escapes = 0
        # 触发脱困时的距离基线：之后距离实质缩小则重置计数
        self.stuck_baseline_d = None
        # 脱困超限后的冷却截止时间（期间不逼近该目标）
        self.stuck_cooldown_until = 0.0
        # 卡死状态所属目标 id（切换目标后整体重置）
        self.stuck_target_id = None

    def estimate_distance(self, area):
        """像素模式：用框面积反推相对距离：distance ≈ C / sqrt(area)"""
        if area <= 0:
            return float("inf")
        return DIST_PER_SQRT_AREA / math.sqrt(area)

    # ---- 方案 19：障碍物避障 ----

    def check_obstacle(self, target, obstacle):
        """检查障碍物是否挡在行驶路径上。

        返回 "clear" / "blocked" / "giveup"：
        - clear: 无障碍或障碍物不在目标与机器人之间
        - blocked: 障碍物挡路但未超时，应停车等待或减速
        - giveup: 持续遮挡超过 OBSTACLE_BLOCK_TIMEOUT，放弃该目标
        """
        if not USE_OBSTACLE_AVOID or obstacle is None:
            self.blocked_since = None
            return "clear"

        if target is None:
            self.blocked_since = None
            return "clear"

        # 障碍物水平方向在目标附近（挡在机器人与目标之间）：
        # 障碍物中心与目标中心水平距离小于两者宽度一半之和
        obs_cx = obstacle["x"] + obstacle["w"] / 2.0
        tgt_cx = target["x"] + target["w"] / 2.0
        horizontal_overlap = (
            abs(obs_cx - tgt_cx)
            < (obstacle["w"] + target["w"]) / 2.0
        )

        # 障碍物是否足够近：
        # 地面模式用投影距离；像素模式用面积近似（面积大 = 近）
        if self.use_ground:
            bearing, obs_dist = self.projection.bearing_and_distance(
                obstacle["x"], obstacle["w"], obstacle["y"], obstacle["h"]
            )
            near = obs_dist <= OBSTACLE_STOP_DISTANCE_M
        else:
            near = obstacle["area"] >= OBSTACLE_AREA_NEAR

        if not (horizontal_overlap and near):
            self.blocked_since = None
            return "clear"

        # 障碍物挡路：开始计时
        now = time.time()

        if self.blocked_since is None:
            self.blocked_since = now

        if now - self.blocked_since >= OBSTACLE_BLOCK_TIMEOUT:
            # 持续遮挡超时：放弃该目标
            self.blocked_since = None
            return "giveup"

        return "blocked"

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

    # ---- 地面投影模式（方案 9 + 26 三段式接近 + 27 卡死脱困） ----

    def _reset_stuck_state(self):
        """方案 27：清空卡死检测状态（换目标/脱困结束/冷却重置时调用）。"""
        self.stuck_history.clear()
        self.stuck_escapes = 0
        self.stuck_baseline_d = None
        self.stuck_target_id = None

    def after_escape(self):
        """方案 27：main 完成一次脱困动作后调用。

        清掉卡死历史（重新开始计时），但保留脱困计数与距离基线，
        以便对同一目标累计脱困次数、判断是否实质脱困。
        """
        self.stuck_history.clear()

    def _check_stuck(self, now, distance, bearing):
        """方案 27：卡死判定。

        记录 (time, distance) 历史，若持续 STUCK_DURATION 秒内距离缩小
        不足 STUCK_DISTANCE_EPS，判定卡死。

        只在目标大致正前方（|bearing| 较小）时才计入样本，避免"原地转
        正方向"期间距离不变被误判卡死。

        返回 True 表示该帧应触发脱困（调用方负责计数/冷却处理）。
        """
        if not (USE_STUCK_DETECT and self.use_ground):
            return False

        if distance >= 999.0:
            # 投影失败（如目标在相机正下方出界），不参与判定
            return False

        # 只有目标大致在正前方才计入"在行驶"：大角度时机器人多半在
        # 原地转正方向，距离本就不该缩小，不算卡死
        if abs(bearing) > 0.5:
            return False

        self.stuck_history.append((now, distance))

        # 保留 STUCK_DURATION + 余量 内的样本
        cutoff = now - STUCK_DURATION - 0.5
        while self.stuck_history and self.stuck_history[0][0] < cutoff:
            self.stuck_history.popleft()

        # 窗口内样本太少（刚开始行驶）不判定
        if len(self.stuck_history) < 2:
            return False

        span = now - self.stuck_history[0][0]

        if span < STUCK_DURATION:
            return False

        d_oldest = self.stuck_history[0][1]
        # 窗口内距离缩小不足阈值 -> 卡死
        return (d_oldest - distance) < STUCK_DISTANCE_EPS

    def _decide_ground(self, target):
        bearing, distance = self.projection.bearing_and_distance(
            target["x"], target["w"], target["y"], target["h"]
        )

        # 更新最后看到的方向（用于丢目标后搜索）
        if bearing < -ANGLE_TOLERANCE:
            self.last_seen_direction = "TURN_LEFT"
        elif bearing > ANGLE_TOLERANCE:
            self.last_seen_direction = "TURN_RIGHT"

        now = time.time()

        # 目标切换：卡死状态与 settle 状态只针对同一目标有效
        tid = target.get("id")
        if self.stuck_target_id is not None and tid is not None \
                and tid != self.stuck_target_id:
            self._reset_stuck_state()

        # 记录当前目标 id（供切换检测用）
        if tid is not None:
            self.stuck_target_id = tid

        # ---- 方案 26：近距判断（蠕行档 + settle） ----
        near = USE_STAGED_APPROACH and distance < STAGED_NEAR_DISTANCE_M
        aligned = abs(bearing) <= ANGLE_TOLERANCE

        # ---- 方案 26：到位 settle 观察 ----
        # 近距且方位角已对齐时，先停车观察 STAGED_SETTLE_TIME 秒确认落位，
        # 再触发 COLLECT，防止投影误差导致没到跟前就开捡
        if USE_STAGED_APPROACH and near and aligned:
            if self.settle_since is None:
                self.settle_since = now
                # 开始 settle 时清掉历史，避免误判卡死（此时在停车）
                self.stuck_history.clear()
                self._limit_accel(0, 0)
                return {"action": "SETTLE"}

            if now - self.settle_since >= STAGED_SETTLE_TIME:
                self.settle_since = None
                # 观察确认落位：进入捡球距离才捡，否则（投影漂移）继续逼近
                if distance <= COLLECT_DISTANCE_M:
                    return {"action": "COLLECT"}
            else:
                # 继续观察中：保持停车
                return {"action": "SETTLE"}

        # 离开近距/对齐状态：重置 settle 计时（重新到位时再观察）
        self.settle_since = None

        # 距离足够近 -> 捡球（非 staged 或 staged 未对齐时的原路径兜底）
        if distance <= COLLECT_DISTANCE_M:
            return {"action": "COLLECT"}

        # ---- 方案 27：脱困冷却期间不逼近，等冷却结束 ----
        if time.time() < self.stuck_cooldown_until:
            self._limit_accel(0, 0)
            return {"action": "SETTLE"}

        # ---- 方案 27：卡死判定（仅在正常行驶中） ----
        if self._check_stuck(now, distance, bearing):
            self.stuck_history.clear()
            self._limit_accel(0, 0)

            if self.stuck_escapes >= STUCK_MAX_ESCAPES:
                # 脱困超限：冷却跳过该目标
                self.stuck_cooldown_until = now + STUCK_RETRY_COOLDOWN
                self._reset_stuck_state()
                return {"action": "SEARCH",
                        "direction": self.last_seen_direction}

            self.stuck_escapes += 1
            self.stuck_baseline_d = distance
            return {"action": "ESCAPE"}

        # ---- 方案 27：距离实质缩小则重置脱困计数（说明在正常接近） ----
        if self.stuck_baseline_d is not None \
                and (self.stuck_baseline_d - distance) >= STUCK_DISTANCE_EPS:
            self.stuck_baseline_d = None
            self.stuck_escapes = 0

        # ---- 方案 26：选择 PD 增益与直行速度档位 ----
        if near:
            kp_a, kd_a = STAGED_KP_ANGLE, STAGED_KD_ANGLE
        else:
            kp_a, kd_a = KP_ANGLE, KD_ANGLE

        p_term = bearing  # 弧度，量级约 -1.5 ~ 1.5

        d_term = 0.0
        if (
            kd_a > 0
            and self.prev_error is not None
            and self.prev_time is not None
            and now > self.prev_time
        ):
            d_term = (bearing - self.prev_error) / (now - self.prev_time)

        self.prev_error = bearing
        self.prev_time = now

        # 差速：bearing 正值 = 目标在右侧 -> 右转
        turn = kp_a * p_term + kd_a * d_term
        turn = max(-1.0, min(1.0, turn))

        # 差速基线速度：近距档用低速，远距档用全速
        base_speed = STAGED_NEAR_SPEED if near else MAX_SPEED

        left = int(base_speed * (1.0 + turn))
        right = int(base_speed * (1.0 - turn))

        left = max(0, min(255, left))
        right = max(0, min(255, right))

        # 方位角已进入死区：直行，速度随距离衰减（近处减速防撞）
        if aligned:
            speed_cap = STAGED_NEAR_SPEED if near else FORWARD_SPEED
            fwd = int(min(speed_cap, distance * FORWARD_DISTANCE_GAIN))
            # 近距蠕行时不再抬到 40（低速更稳），远距保持下限
            floor = 20 if near else 40
            fwd = max(fwd, floor)
            left = fwd
            right = fwd

        # 保证低速时电机仍能转动（近距蠕行档跳过抬升，允许更慢）
        if not near:
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
