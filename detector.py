import time

from ultralytics import YOLO

from config import (
    YOLO_MODEL_PATH,
    CONFIDENCE_THRESHOLD,
    USE_TRACKER,
    EMA_ALPHA,
    LOCK_TARGET_ID,
    USE_KALMAN,
    KALMAN_POS_NOISE,
    KALMAN_VEL_NOISE,
    KALMAN_MEAS_NOISE,
    USE_OBSTACLE_AVOID,
    OBSTACLE_MODEL_PATH,
    OBSTACLE_CLASSES,
    OBSTACLE_CONF_THRESHOLD
)

from kalman_filter import ConstantVelocityKalman


class ShuttleDetector:
    """
    方案 4（ByteTrack 跟踪）+ 方案 8（Kalman 平滑）+ 方案 19（障碍物检测）。

    - model.track 获得稳定的目标 ID，位置逐帧关联，单帧检测噪声被跟踪器吸收
    - 位置 (cx, cy) 用匀速模型 Kalman 平滑；漏检期间 predict() 外推轨迹，
      最多外推 KALMAN_MAX_PREDICT_S 秒；w/h/area 仍用 EMA 平滑
    - USE_KALMAN=False 时退回原 EMA 全量平滑
    - 单帧漏检时返回外推/回放结果并标记 "missed": True，
      丢帧计数由 DecisionMaker 管理

    detect() 返回格式：
    {
        "x", "y", "w", "h", "area", "confidence", "score",
        "id": 跟踪 ID（未启用跟踪时为 None）,
        "missed": 是否为漏检时的外推/回放结果
    }
    第二个返回值 mask 不再使用，恒为 None。
    """

    # 漏检期间最多外推的秒数，超过后不再外推（防止轨迹飞掉）
    KALMAN_MAX_PREDICT_S = 1.0

    def __init__(self):
        self.model = YOLO(YOLO_MODEL_PATH)

        # 方案 19：障碍物检测模型（可选，为 None 时复用羽毛球模型）
        self.obstacle_model = None

        if USE_OBSTACLE_AVOID:
            path = OBSTACLE_MODEL_PATH or YOLO_MODEL_PATH
            self.obstacle_model = YOLO(path)

            self.obstacle_class_ids = None

            names = self.obstacle_model.names

            if names:
                wanted = set(OBSTACLE_CLASSES)
                self.obstacle_class_ids = {
                    cid for cid, name in names.items() if name in wanted
                }

        # EMA 平滑后的目标状态（w/h/area 一直用 EMA；位置在非 Kalman 模式下也用它）
        self.smoothed = None
        # 锁定的跟踪 ID
        self.locked_id = LOCK_TARGET_ID

        # Kalman 状态（位置平滑 + 速度估计）
        self.kalman = ConstantVelocityKalman(
            KALMAN_POS_NOISE, KALMAN_VEL_NOISE, KALMAN_MEAS_NOISE
        )
        self.last_meas_time = None

        # 本帧全部候选框（不做平滑/选择），供捡球确认和多球调度使用
        self.last_candidates = []

        # 方案 19：本帧最近的障碍物框（供决策避障和调试显示用）
        self.last_obstacle = None

    def detect_obstacles(self, frame):
        """方案 19：检测障碍物，返回本帧最近的障碍物框（无则 None）。

        障碍物框格式与候选框一致，额外带 "cls" 类别名。
        "最近"按框底边 y 值最大（画面中越靠下越近）判断。
        """
        self.last_obstacle = None

        if not USE_OBSTACLE_AVOID or self.obstacle_model is None:
            return None

        results = self.obstacle_model(
            frame,
            conf=OBSTACLE_CONF_THRESHOLD,
            verbose=False
        )

        result = results[0]

        if result.boxes is None or len(result.boxes) == 0:
            return None

        xyxys = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        clss = result.boxes.cls.cpu().numpy()

        nearest = None

        for i in range(len(xyxys)):
            cls_id = int(clss[i])

            # 只保留配置的障碍物类别
            if (
                self.obstacle_class_ids is not None
                and cls_id not in self.obstacle_class_ids
            ):
                continue

            x1, y1, x2, y2 = xyxys[i]
            w = int(x2 - x1)
            h = int(y2 - y1)

            if w <= 0 or h <= 0:
                continue

            box = {
                "x": int(x1),
                "y": int(y1),
                "w": w,
                "h": h,
                "area": w * h,
                "confidence": float(confs[i]),
                "cls": self.obstacle_model.names.get(cls_id, str(cls_id))
            }

            # 越靠下（底边 y 越大）越近
            if nearest is None or box["y"] + box["h"] > nearest["y"] + nearest["h"]:
                nearest = box

        self.last_obstacle = nearest

        return nearest

    def _ema_update(self, prev, cur):
        """对框尺寸和面积做指数滑动平均。"""
        a = EMA_ALPHA
        if prev is None:
            return dict(cur)

        smoothed = dict(cur)
        for key in ("w", "h", "area"):
            smoothed[key] = prev[key] * (1.0 - a) + cur[key] * a
        return smoothed

    def _measure(self, cx, cy):
        """用新测量更新 Kalman，返回平滑后的 (cx, cy)。"""
        now = time.time()

        if self.last_meas_time is None:
            dt = 0.033
        else:
            dt = min(max(now - self.last_meas_time, 1e-3), 0.5)

        self.last_meas_time = now
        return self.kalman.update(cx, cy, dt)

    def _extrapolate(self):
        """漏检期间外推目标位置；无历史或外推超时返回 None。"""
        if not USE_KALMAN or self.smoothed is None:
            return None

        if self.last_meas_time is None:
            return None

        dt = time.time() - self.last_meas_time

        if dt > self.KALMAN_MAX_PREDICT_S:
            return None

        pos = self.kalman.predict(dt)

        if pos is None:
            return None

        cx, cy = pos
        extrapolated = dict(self.smoothed)
        extrapolated["x"] = int(cx - extrapolated["w"] / 2.0)
        extrapolated["y"] = int(cy - extrapolated["h"])
        extrapolated["predicted"] = True

        return extrapolated

    def detect(self, frame):
        if USE_TRACKER:
            results = self.model.track(
                frame,
                conf=CONFIDENCE_THRESHOLD,
                persist=True,
                tracker="bytetrack.yaml",
                verbose=False
            )
        else:
            results = self.model(
                frame,
                conf=CONFIDENCE_THRESHOLD,
                verbose=False
            )

        result = results[0]

        if result.boxes is None or len(result.boxes) == 0:
            # 单帧漏检：优先外推轨迹，否则回放旧结果并标记 missed
            self.last_candidates = []
            extrapolated = self._extrapolate()

            if extrapolated is not None:
                return extrapolated, None

            if self.smoothed is not None:
                stale = dict(self.smoothed)
                stale["missed"] = True
                return stale, None
            return None, None

        # 一次性取所有框，避免逐框 GPU->CPU 同步
        xyxys = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        ids = (
            result.boxes.id.cpu().numpy()
            if result.boxes.id is not None
            else None
        )

        candidates = []

        for i in range(len(xyxys)):
            x1, y1, x2, y2 = xyxys[i]
            confidence = float(confs[i])

            x = int(x1)
            y = int(y1)
            w = int(x2 - x1)
            h = int(y2 - y1)

            if w <= 0 or h <= 0:
                continue

            track_id = int(ids[i]) if ids is not None else None

            candidates.append({
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "area": w * h,
                "confidence": confidence,
                "score": confidence * w * h,
                "id": track_id
            })

        # 保存本帧全部候选框（供捡球确认和多球调度使用）
        self.last_candidates = candidates

        if len(candidates) == 0:
            extrapolated = self._extrapolate()

            if extrapolated is not None:
                return extrapolated, None

            if self.smoothed is not None:
                stale = dict(self.smoothed)
                stale["missed"] = True
                return stale, None
            return None, None

        # 目标选择：锁定 ID 优先，否则选分数最高
        if self.locked_id is not None:
            locked = [
                c for c in candidates if c["id"] == self.locked_id
            ]
            best = locked[0] if locked else max(
                candidates, key=lambda item: item["score"]
            )
        else:
            best = max(candidates, key=lambda item: item["score"])
            # 自动锁定第一个跟踪到的 ID（仅启用跟踪且拿到 ID 时）
            if best["id"] is not None:
                self.locked_id = best["id"]

        self.smoothed = self._ema_update(self.smoothed, best)

        # 位置平滑：Kalman（可外推）或退回 EMA 全量
        cx = best["x"] + best["w"] / 2.0
        cy = best["y"] + best["h"]

        if USE_KALMAN:
            scx, scy = self._measure(cx, cy)
        else:
            scx = self.smoothed["x"] * (1.0 - EMA_ALPHA) + cx * EMA_ALPHA
            scy = self.smoothed["y"] * (1.0 - EMA_ALPHA) + cy * EMA_ALPHA

        self.smoothed["x"] = int(scx - self.smoothed["w"] / 2.0)
        self.smoothed["y"] = int(scy - self.smoothed["h"])

        output = dict(self.smoothed)
        output["missed"] = False

        return output, None

    def aim_at(self, box):
        """多球调度（方案 10）：把跟踪焦点切到指定候选框。

        - 有跟踪 ID 时锁定该框的 ID（ByteTrack 继续稳定关联）
        - 无 ID（未启用跟踪）时用 Kalman 直接重置到新框中心，
          避免旧速度外拉
        """
        self.locked_id = box["id"]

        cx = box["x"] + box["w"] / 2.0
        cy = box["y"] + box["h"]

        if USE_KALMAN:
            self.kalman.reset_to(cx, cy)

        self.smoothed = dict(box)
        self.smoothed["x"] = int(cx - box["w"] / 2.0)
        self.smoothed["y"] = int(cy - box["h"])

    def reset(self):
        """丢帧超限后由决策层调用：清空平滑状态和 ID 锁定。"""
        self.smoothed = None
        self.locked_id = LOCK_TARGET_ID
        self.kalman = ConstantVelocityKalman(
            KALMAN_POS_NOISE, KALMAN_VEL_NOISE, KALMAN_MEAS_NOISE
        )
        self.last_meas_time = None
        self.last_candidates = []
