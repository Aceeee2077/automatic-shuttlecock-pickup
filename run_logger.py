"""
全流程运行日志器。

主循环每一帧存一张 JPEG + 一行 JSON（检测框、动作指令、丢帧计数、
电量、避障状态等），关键动作（捡球开始/成功/失败、脱困、搜索、
低电量等）单独写事件流。产出 run 目录，供 replay_viewer.py 离线回放。
"""

import json
import queue
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2


class RunLogger:
    """后台线程异步落盘，不阻塞主循环。

    run 目录结构：
        frames/*.jpg     按帧序编号的 JPEG
        log.jsonl        每帧一条记录（t=相对秒、frame、target、action...）
        events.jsonl     事件流（collect_start / escape / battery_low...）
        meta.json        运行元信息（起止时间、帧数）
    """

    def __init__(self, root_dir="runs/replay", jpeg_quality=85,
                 every_n_frames=1):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = Path(root_dir) / f"run_{ts}"
        self.frames_dir = self.run_dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)

        self.log_path = self.run_dir / "log.jsonl"
        self.events_path = self.run_dir / "events.jsonl"

        self.jpeg_quality = int(jpeg_quality)
        self.every_n = max(1, int(every_n_frames))

        self._start_wall = time.time()
        self._logged = 0
        self._seq = 0
        self._event_count = 0
        self._q = queue.Queue()
        self._writer = threading.Thread(
            target=self._write_loop, daemon=True
        )
        self._writer.start()

        self.event("run_start")

    def _write_loop(self):
        with open(self.log_path, "a", encoding="utf-8") as lf, \
                open(self.events_path, "a", encoding="utf-8") as ef:
            while True:
                item = self._q.get()
                if item is None:
                    break

                kind, payload = item

                if kind == "frame":
                    jpeg_bytes, record = payload
                    self._seq += 1
                    name = f"{self._seq:06d}.jpg"
                    (self.frames_dir / name).write_bytes(jpeg_bytes)
                    record["frame"] = name
                    record["seq"] = self._seq
                    lf.write(json.dumps(record, ensure_ascii=False) + "\n")

                elif kind == "event":
                    self._event_count += 1
                    payload["seq_at"] = self._seq
                    ef.write(json.dumps(payload, ensure_ascii=False) + "\n")

                elif kind == "meta":
                    payload["frame_count"] = self._seq
                    payload["event_count"] = self._event_count
                    (self.run_dir / "meta.json").write_text(
                        json.dumps(payload, ensure_ascii=False, indent=2),
                        encoding="utf-8"
                    )

            lf.flush()
            ef.flush()

    def log_frame(self, frame, record):
        """主循环每帧调用：编码 JPEG 并入队，由后台线程落盘。

        record 里不要带 frame 字段，帧文件名由后台线程统一编号。
        """
        self._logged += 1

        if self._logged % self.every_n != 0:
            return

        ok, buf = cv2.imencode(
            ".jpg", frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )

        if not ok:
            return

        rec = {"t": round(time.time() - self._start_wall, 3)}
        rec.update(record)
        self._q.put(("frame", (buf.tobytes(), rec)))

    def event(self, name, **data):
        """记录一个关键事件（捡球、脱困、低电量等）。"""
        payload = {
            "t": round(time.time() - self._start_wall, 3),
            "event": name,
        }
        payload.update(data)
        self._q.put(("event", payload))

    def close(self):
        """flush 剩余数据、写 meta.json、停后台线程（可重复调用）。"""
        if getattr(self, "_closed", False):
            return

        self._closed = True
        self.event("run_stop")
        self._q.put(("meta", {
            "start_time": self._start_wall,
            "end_time": time.time(),
            "created": datetime.now().isoformat(timespec="seconds"),
        }))
        self._q.put(None)
        self._writer.join(timeout=10.0)
        print(f"Run log saved: {self.run_dir}")
