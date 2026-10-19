"""
运行日志离线回放查看器。

读取 run_logger.py 产出的 run 目录（frames/*.jpg + log.jsonl + events.jsonl），
按录制时间轴回放每一帧（日志帧已带现场调试标注），叠加显示当时的动作、
目标框、距离、电量等状态和事件提示，用于事后排查捡球失败、卡死脱困、
丢目标等问题。

用法：
    python replay_viewer.py                     # 回放最新一次 run
    python replay_viewer.py --run <run目录>      # 指定 run 目录
    python replay_viewer.py --speed 2           # 2 倍速
    python replay_viewer.py --start 35          # 从第 35 秒开始

按键：
    空格        播放 / 暂停
    , .         后退 1 帧 / 前进 1 帧
    0~9         跳转到 0%~90% 进度
    [ ]         倍速减半 / 加倍
    e E         跳到下一条 / 上一条事件（并暂停）
    拖动条       任意位置 seek
    q           退出
"""

import argparse
import bisect
import json
import sys
from pathlib import Path

import cv2


def find_latest_run(root_dir):
    """返回 root_dir 下最新的 run 目录；不存在则返回 None。"""
    root = Path(root_dir)

    if not root.is_dir():
        return None

    runs = sorted(
        [p for p in root.iterdir() if p.is_dir() and (p / "log.jsonl").is_file()]
    )
    return runs[-1] if runs else None


def load_run(run_dir):
    """加载 run 目录：逐帧记录 + 事件流，返回 (frames, events)。"""
    frames = []
    log_path = Path(run_dir) / "log.jsonl"

    with open(log_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()

            if line:
                frames.append(json.loads(line))

    events = []
    events_path = Path(run_dir) / "events.jsonl"

    if events_path.is_file():
        with open(events_path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()

                if line:
                    events.append(json.loads(line))

    return frames, events


def _fmt_time(t):
    """秒 -> mm:ss.d"""
    m = int(t // 60)
    s = t - m * 60
    return f"{m:02d}:{s:04.1f}"


def _status_text(rec):
    """把一帧记录拼成回放状态栏文本。"""
    parts = [f"t={_fmt_time(rec.get('t', 0))}"]

    action = rec.get("action", "?")
    parts.append(action)

    if "left" in rec and "right" in rec:
        parts.append(f"L:{rec['left']} R:{rec['right']}")

    tgt = rec.get("target")

    if tgt:
        tag = "STALE " if tgt.get("missed") else ""
        parts.append(
            f"box=({tgt['x']},{tgt['y']},{tgt['w']}x{tgt['h']}) "
            f"{tag}{tgt['conf']:.2f}"
        )

    if "distance_m" in rec:
        parts.append(f"d={rec['distance_m']:.2f}m a={rec.get('bearing', 0) * 57.3:.0f}deg")

    parts.append(f"miss={rec.get('miss', 0)}")
    parts.append(f"got={rec.get('collected', 0)}")

    batt = rec.get("battery_mv")

    if batt:
        parts.append(f"batt={batt / 1000.0:.2f}V")

    obs = rec.get("obstacle_state", "clear")

    if obs != "clear":
        parts.append(f"OBSTACLE:{obs}")

    return "  ".join(parts)


def _event_label(ev):
    name = ev.get("event", "?")
    extra = ""

    if name == "battery_low":
        extra = f" {ev.get('mv', '')}mV"

    if "collected" in ev:
        extra = f" #{ev['collected']}"

    return f"{_fmt_time(ev.get('t', 0))}  {name}{extra}"


class FrameCache:
    """只缓存最近一帧，避免同一帧重复读盘。"""

    def __init__(self, run_dir):
        self.run_dir = Path(run_dir)
        self._name = None
        self._img = None

    def get(self, frame_name):
        if self._name != frame_name:
            self._img = cv2.imread(str(self.run_dir / "frames" / frame_name))
            self._name = frame_name

        return self._img


def _find_idx_at(frames, t):
    """找到时间轴上 <= t 的最近一帧索引；t 早于第一帧返回 0。"""
    idx = bisect.bisect_right([fr["t"] for fr in frames], t)
    return max(0, idx - 1)


def run_viewer(run_dir, speed=1.0, start_t=0.0):
    run_dir = Path(run_dir)
    frames, events = load_run(run_dir)

    if not frames:
        print(f"No frames in {run_dir}")
        return 1

    total_t = frames[-1]["t"]
    timestamps = [fr["t"] for fr in frames]
    cache = FrameCache(run_dir)

    win = "Replay"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    TRACKBAR = "Time"
    cv2.createTrackbar(TRACKBAR, win, 0, 1000, lambda _x: None)

    playing = True
    cur_t = min(start_t, total_t)
    event_cursor = 0
    event_times = [ev.get("t", 0.0) for ev in events]
    _last_shown = [None, None]  # [frame_name, img]

    def seek_event(delta):
        nonlocal event_cursor, cur_t, playing

        if not events:
            print("No events in this run.")
            return

        if event_cursor == 0 or event_times[event_cursor - 1] > cur_t + 0.01:
            # 定位到当前时间附近（bisect 要求有序列表）
            event_cursor = bisect.bisect_right(event_times, cur_t)

        event_cursor = max(0, min(len(events) - 1, event_cursor + delta))
        cur_t = event_times[event_cursor]
        playing = False
        cv2.setTrackbarPos(TRACKBAR, win, int(cur_t / total_t * 1000))

    while True:
        key = cv2.waitKey(30) & 0xFF

        if key == ord("q"):
            break

        if key == ord(" "):
            playing = not playing
        elif key == ord(","):
            idx = max(0, _find_idx_at(frames, cur_t) - 1)
            cur_t = timestamps[idx]
            playing = False
        elif key == ord("."):
            idx = min(len(frames) - 1, _find_idx_at(frames, cur_t) + 1)
            cur_t = timestamps[idx] + 0.001
            playing = False
        elif 0x30 <= key <= 0x39:
            cur_t = total_t * (key - 0x30) / 10.0
            playing = False
        elif key == ord("["):
            speed = max(0.25, speed / 2)
        elif key == ord("]"):
            speed = min(8.0, speed * 2)
        elif key == ord("e"):
            seek_event(1)
        elif key == ord("E"):
            seek_event(-1)

        # 拖动条位置 -> 时间
        pos = cv2.getTrackbarPos(TRACKBAR, win)
        bar_t = pos / 1000.0 * total_t

        if abs(bar_t - cur_t) > 1e-6:
            cur_t = bar_t
            playing = False

        if playing:
            cur_t += 0.03 * speed

        if cur_t > total_t:
            cur_t = total_t
            playing = False

        idx = _find_idx_at(frames, cur_t)
        rec = frames[idx]
        img = cache.get(rec["frame"])

        if img is None:
            print(f"Missing frame file: {run_dir / 'frames' / rec['frame']}")
            continue

        h, w = img.shape[:2]

        # 顶部状态栏
        bar = 28
        cv2.rectangle(img, (0, 0), (w, bar), (0, 0, 0), -1)
        cv2.putText(
            img, _status_text(rec), (10, bar - 9),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1
        )
        cv2.putText(
            img, f"[{speed:.2f}x] {'PLAY' if playing else 'PAUSE'}",
            (w - 260, bar - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1
        )

        # 底部：当前时间前后 2 秒内的事件
        recent = [ev for ev in events if abs(ev.get("t", 0) - cur_t) <= 2.0]

        for i, ev in enumerate(recent):
            cv2.putText(
                img, "EVENT: " + _event_label(ev),
                (10, h - 30 + i * 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1
            )

        cv2.putText(
            img,
            f"{_fmt_time(cur_t)} / {_fmt_time(total_t)}  (frame {idx + 1}/{len(frames)})",
            (w - 380, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1
        )

        cv2.imshow(win, img)

    cv2.destroyAllWindows()
    return 0


def main():
    parser = argparse.ArgumentParser(description="Replay a recorded robot run")
    parser.add_argument(
        "--run", default=None,
        help="run directory (default: latest under runs/replay)"
    )
    parser.add_argument("--speed", type=float, default=1.0, help="playback speed")
    parser.add_argument("--start", type=float, default=0.0, help="start time (s)")
    args = parser.parse_args()

    run_dir = Path(args.run) if args.run else find_latest_run("runs/replay")

    if run_dir is None or not (run_dir / "log.jsonl").is_file():
        print("No recorded run found. Run main.py with USE_RUN_LOGGER=True first.")
        return 1

    print(f"Replaying: {run_dir}")
    print("Space=play/pause  , . =step  [ ]=speed  0-9=seek  e/E=events  q=quit")
    return run_viewer(run_dir, speed=args.speed, start_t=args.start)


if __name__ == "__main__":
    sys.exit(main())
