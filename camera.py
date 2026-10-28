import threading
import time

import cv2

from camera_watchdog import FrameWatchdog
from config import (
    CAMERA_INDEX,
    CAMERA_STALE_TIMEOUT,
    FRAME_WIDTH,
    FRAME_HEIGHT
)


class Camera:
    """
    后台线程读帧。

    推理慢时相机缓冲会积压，串行 read() 拿到的是几百毫秒前的旧画面。
    改为后台线程持续 grab() 并只保留最新一帧，主循环 retrieve() 时
    永远拿到最近一帧，感知延迟从「一个推理周期」降到「一帧」。
    """

    def __init__(self):
        self.cap = cv2.VideoCapture(CAMERA_INDEX)

        if not self.cap.isOpened():
            raise RuntimeError(
                "Cannot open camera. Try changing CAMERA_INDEX in config.py."
            )

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        self.lock = threading.Lock()
        self.frame = None
        self.stopped = False
        self.watchdog = FrameWatchdog(CAMERA_STALE_TIMEOUT)

        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        """后台线程：持续读帧，只保留最新一帧。"""
        while not self.stopped:
            ret, frame = self.cap.read()

            if not ret:
                # 读帧失败：短暂等待后重试，多次失败则视为相机断开
                with self.lock:
                    self.frame = None
                # 失败时不要空转，否则读失败会占满一个 CPU 核
                time.sleep(0.02)
                continue

            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

            with self.lock:
                self.frame = frame
                self.watchdog.mark_frame()

    def read(self, timeout=2.0):
        """取最新一帧；启动后还没读到首帧或相机断开时返回 None。"""
        deadline = time.time() + timeout

        while time.time() < deadline:
            with self.lock:
                frame = self.frame

            if frame is not None:
                return frame

            time.sleep(0.01)

        return None

    def frame_age(self):
        """距上一帧新画面的秒数；还没收到首帧时返回 None。"""
        return self.watchdog.frame_age()

    def is_stale(self):
        """画面超过 CAMERA_STALE_TIMEOUT 没有新帧。"""
        return self.watchdog.is_stale()

    def release(self):
        self.stopped = True
        self.thread.join(timeout=1.0)
        self.cap.release()
