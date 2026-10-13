from ultralytics import YOLO

from config import (
    YOLO_MODEL_PATH,
    CONFIDENCE_THRESHOLD,
    USE_TRACKER,
    EMA_ALPHA,
    LOCK_TARGET_ID
)


class ShuttleDetector:
    """
    方案 4：ByteTrack 跟踪 + EMA 平滑。

    - model.track 获得稳定的目标 ID，位置逐帧关联，单帧检测噪声被跟踪器吸收
    - 框位置和面积做指数滑动平均（EMA），消除逐帧抖动
    - 单帧漏检时返回上一帧平滑结果并标记 "missed": True，
      丢帧计数由 DecisionMaker 管理

    detect() 返回格式：
    {
        "x", "y", "w", "h", "area", "confidence", "score",
        "id": 跟踪 ID（未启用跟踪时为 None）,
        "missed": 是否为漏检时回放的旧结果
    }
    第二个返回值 mask 不再使用，恒为 None。
    """

    def __init__(self):
        self.model = YOLO(YOLO_MODEL_PATH)

        # EMA 平滑后的目标状态
        self.smoothed = None
        # 锁定的跟踪 ID
        self.locked_id = LOCK_TARGET_ID

    def _ema_update(self, prev, cur):
        """对框位置和面积做指数滑动平均。"""
        a = EMA_ALPHA
        if prev is None:
            return dict(cur)

        smoothed = dict(cur)
        for key in ("x", "y", "w", "h", "area"):
            smoothed[key] = prev[key] * (1.0 - a) + cur[key] * a
        return smoothed

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
            # 单帧漏检：回放上一帧平滑结果并标记 missed，决策层按丢帧计数处理
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

        if len(candidates) == 0:
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

        output = dict(self.smoothed)
        output["missed"] = False

        return output, None

    def reset(self):
        """丢帧超限后由决策层调用：清空平滑状态和 ID 锁定。"""
        self.smoothed = None
        self.locked_id = LOCK_TARGET_ID
