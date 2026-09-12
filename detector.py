from ultralytics import YOLO

from config import YOLO_MODEL_PATH, CONFIDENCE_THRESHOLD


class ShuttleDetector:
    def __init__(self):
        self.model = YOLO(YOLO_MODEL_PATH)

    def detect(self, frame):
        """
        YOLO 羽毛球检测版本。

        返回格式和之前 OpenCV 版本保持一致：
        {
            "x": 左上角 x,
            "y": 左上角 y,
            "w": 宽度,
            "h": 高度,
            "area": 框面积,
            "confidence": 置信度,
            "score": 分数
        }

        第二个返回值 mask 现在不再使用，所以返回 None。
        """

        results = self.model(
            frame,
            conf=CONFIDENCE_THRESHOLD,
            verbose=False
        )

        result = results[0]

        candidates = []

        if result.boxes is None:
            return None, None

        for box in result.boxes:
            xyxy = box.xyxy[0].cpu().numpy()
            confidence = float(box.conf[0].cpu().numpy())

            x1, y1, x2, y2 = xyxy

            x = int(x1)
            y = int(y1)
            w = int(x2 - x1)
            h = int(y2 - y1)

            if w <= 0 or h <= 0:
                continue

            area = w * h

            candidates.append({
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "area": area,
                "confidence": confidence,
                "score": confidence * area
            })

        if len(candidates) == 0:
            return None, None

        # 选择分数最高的羽毛球
        best = max(candidates, key=lambda item: item["score"])

        return best, None