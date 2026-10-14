"""
YOLO 框标注工具（单类：羽毛球）。

读取 dataset/images/*.jpg，用鼠标画框标注，存为 YOLO 格式
dataset/labels/<图片名>.txt（每行：0 cx cy w h，归一化 0~1）。

操作：
  鼠标左键拖拽      画新框（或拖动已有框 / 拖动角点缩放）
  S                 保存当前图标注
  N / P             下一张 / 上一张（未保存会询问）
  D                 删除选中的框
  U                 撤销上一步
  Q                 退出（未保存会询问）

用法：
    python label_tool.py                 # 从第一张开始
    python label_tool.py --start 120     # 从第 120 张开始（断点续标）

注意：标注框尽量紧贴球的轮廓；一张图有多颗球就画多个框。
"""

import argparse
from pathlib import Path

import cv2

ROOT = Path(__file__).parent
IMAGES_DIR = ROOT / "dataset" / "images"
LABELS_DIR = ROOT / "dataset" / "labels"

GREEN = (0, 255, 0)
YELLOW = (0, 220, 255)
BLUE = (255, 128, 0)
RED = (0, 0, 255)

HANDLE_RADIUS = 6


def load_boxes(label_file: Path):
    """读取 YOLO 格式标注，返回像素坐标框列表 [(x1, y1, x2, y2), ...]。"""
    boxes = []
    if not label_file.exists():
        return boxes
    h, w = _IMAGE_SIZE[1], _IMAGE_SIZE[0]
    for line in label_file.read_text().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        _, cx, cy, bw, bh = (float(v) for v in parts[:5])
        x1 = (cx - bw / 2) * w
        y1 = (cy - bh / 2) * h
        x2 = (cx + bw / 2) * w
        y2 = (cy + bh / 2) * h
        boxes.append((float(x1), float(y1), float(x2), float(y2)))
    return boxes


def save_boxes(label_file: Path, boxes):
    """把像素坐标框写成 YOLO 格式（单类，class id = 0）。"""
    h, w = _IMAGE_SIZE[1], _IMAGE_SIZE[0]
    lines = []
    for x1, y1, x2, y2 in boxes:
        x1, x2 = max(0.0, min(float(x1), w)), max(0.0, min(float(x2), w))
        y1, y2 = max(0.0, min(float(y1), h)), max(0.0, min(float(y2), h))
        if x2 - x1 < 2 or y2 - y1 < 2:
            continue
        cx, cy = (x1 + x2) / 2 / w, (y1 + y2) / 2 / h
        bw, bh = (x2 - x1) / w, (y2 - y1) / h
        lines.append(f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
    LABELS_DIR.mkdir(parents=True, exist_ok=True)
    if lines:
        label_file.write_text("\n".join(lines) + "\n")
    elif label_file.exists():
        label_file.unlink()


_IMAGE_SIZE = (0, 0)  # (w, h)，加载图片后更新


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


class Labeler:
    def __init__(self, start_index):
        self.images = sorted(IMAGES_DIR.glob("*.jpg"))
        if not self.images:
            raise RuntimeError(f"dataset/images 下没有图片，先运行 capture_dataset.py")
        self.index = clamp(start_index, 0, len(self.images) - 1)
        self.image = self._load_image()
        global _IMAGE_SIZE
        h, w = self.image.shape[:2]
        _IMAGE_SIZE = (w, h)

        self.boxes = load_boxes(self._label_file())
        self.selected = None          # 选中框的下标
        self.dirty = False            # 当前图有未保存修改
        self.undo_stack = []          # [(boxes, selected), ...]

        # 鼠标交互状态
        self.mode = None              # drawing / moving / resizing
        self.drag_start = None
        self.drag_box_backup = None
        self.resize_corner = None
        self.move_offset = None

        self._setup_mouse()

    # ---------- 图片 / 标注文件 ----------

    def _label_file(self):
        return LABELS_DIR / (self.images[self.index].stem + ".txt")

    def _load_image(self):
        return cv2.imread(str(self.images[self.index]))

    # ---------- 鼠标 ----------

    def _setup_mouse(self):
        cv2.namedWindow("Label")
        cv2.setMouseCallback("Label", self._on_mouse)

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._on_down(x, y)
        elif event == cv2.EVENT_MOUSEMOVE:
            self._on_move(x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self._on_up(x, y)

    def _find_box(self, x, y):
        """返回 (框下标, 角点编号)：角点优先，其次框内（最上层）。"""
        for i in range(len(self.boxes) - 1, -1, -1):
            x1, y1, x2, y2 = self.boxes[i]
            for corner, cx, cy in (
                (0, x1, y1), (1, x2, y1), (2, x2, y2), (3, x1, y2)
            ):
                if abs(x - cx) <= HANDLE_RADIUS and abs(y - cy) <= HANDLE_RADIUS:
                    return i, corner
        for i in range(len(self.boxes) - 1, -1, -1):
            x1, y1, x2, y2 = self.boxes[i]
            if x1 <= x <= x2 and y1 <= y <= y2:
                return i, None
        return None, None

    def _on_down(self, x, y):
        idx, corner = self._find_box(x, y)
        self._push_undo()
        self.dirty = True

        if idx is not None and corner is None:
            self.selected = idx
            self.mode = "moving"
            x1, y1, _, _ = self.boxes[idx]
            self.move_offset = (x - x1, y - y1)
        elif idx is not None:
            self.selected = idx
            self.mode = "resizing"
            self.resize_corner = corner
        else:
            self.selected = None
            self.mode = "drawing"
            self.drag_start = (x, y)
            self.drag_box_backup = (x, y)

    def _on_move(self, x, y):
        if self.mode == "drawing" and self.drag_start:
            sx, sy = self.drag_start
            self.drag_box_backup = (sx, sy, x, y)
        elif self.mode == "moving" and self.selected is not None:
            h, w = self.image.shape[:2]
            x1, y1, x2, y2 = self.boxes[self.selected]
            bw, bh = x2 - x1, y2 - y1
            nx1 = clamp(x - self.move_offset[0], 0, w - bw)
            ny1 = clamp(y - self.move_offset[1], 0, h - bh)
            self.boxes[self.selected] = (nx1, ny1, nx1 + bw, ny1 + bh)
        elif self.mode == "resizing" and self.selected is not None:
            h, w = self.image.shape[:2]
            x1, y1, x2, y2 = self.boxes[self.selected]
            corner = self.resize_corner
            if corner == 0:
                x1, y1 = clamp(x, 0, x2 - 2), clamp(y, 0, y2 - 2)
            elif corner == 1:
                x2, y1 = clamp(x, x1 + 2, w), clamp(y, 0, y2 - 2)
            elif corner == 2:
                x2, y2 = clamp(x, x1 + 2, w), clamp(y, y1 + 2, h)
            else:
                x1, y2 = clamp(x, 0, x2 - 2), clamp(y, y1 + 2, h)
            self.boxes[self.selected] = (x1, y1, x2, y2)

    def _on_up(self, x, y):
        if self.mode == "drawing":
            sx, sy, ex, ey = self.drag_box_backup
            if abs(ex - sx) < 8 or abs(ey - sy) < 8:
                # 拖动太小，视为误点：撤销并取消选中
                self._pop_undo()
                self.dirty = False
                self.selected = None
            else:
                self.boxes.append(
                    (min(sx, ex), min(sy, ey), max(sx, ex), max(sy, ey))
                )
                self.selected = len(self.boxes) - 1
        self.mode = None
        self.drag_start = None

    # ---------- 状态管理 ----------

    def _push_undo(self):
        self.undo_stack.append((list(self.boxes), self.selected))
        if len(self.undo_stack) > 50:
            self.undo_stack.pop(0)

    def _pop_undo(self):
        if self.undo_stack:
            self.boxes, self.selected = self.undo_stack.pop()

    def _goto(self, new_index):
        if self.dirty:
            self._save_quiet()
            print(f"已自动保存 {self._label_file().name}")
        self.index = clamp(new_index, 0, len(self.images) - 1)
        self.image = self._load_image()
        h, w = self.image.shape[:2]
        global _IMAGE_SIZE
        _IMAGE_SIZE = (w, h)
        self.boxes = load_boxes(self._label_file())
        self.selected = None
        self.dirty = False
        self.undo_stack = []

    def _save_quiet(self):
        save_boxes(self._label_file(), self.boxes)
        self.dirty = False

    @staticmethod
    def _confirm(prompt):
        key = cv2.waitKeyEx(0) & 0xFFFF
        return key in (ord("y"), 13)

    # ---------- 主循环 ----------

    def run(self):
        while True:
            display = self.image.copy()

            for i, (x1, y1, x2, y2) in enumerate(self.boxes):
                color = GREEN if i == self.selected else YELLOW
                pt1, pt2 = (int(x1), int(y1)), (int(x2), int(y2))
                cv2.rectangle(display, pt1, pt2, color, 2)
                if i == self.selected:
                    for cx, cy in (pt1, (int(x2), int(y1)), pt2, (int(x1), int(y2))):
                        cv2.circle(display, (cx, cy), 4, RED, -1)
            if self.mode == "drawing" and self.drag_box_backup and len(self.drag_box_backup) == 4:
                sx, sy, ex, ey = (int(v) for v in self.drag_box_backup)
                cv2.rectangle(display, (sx, sy), (ex, ey), BLUE, 1)

            status = (
                f"[{self.index + 1}/{len(self.images)}] {self.images[self.index].name} | "
                f"boxes: {len(self.boxes)}"
                + (" | *unsaved" if self.dirty else "")
                + " | L-drag: new/move/resize  S:save  N/P:nav  D:del  U:undo  Q:quit"
            )
            cv2.putText(display, status, (10, display.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("Label", display)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                if self.dirty:
                    self._save_quiet()
                    print(f"已自动保存 {self._label_file().name}")
                break
            elif key == ord("s"):
                self._save_quiet()
                print(f"已保存 {self._label_file().name}（{len(self.boxes)} 个框）")
            elif key == ord("n"):
                self._goto(self.index + 1)
            elif key == ord("p"):
                self._goto(self.index - 1)
            elif key == ord("d") and self.selected is not None:
                self._push_undo()
                self.boxes.pop(self.selected)
                self.selected = None
                self.dirty = True
            elif key == ord("u"):
                self._pop_undo()
                self.dirty = True

        cv2.destroyAllWindows()
        print("标注结束")


def main():
    parser = argparse.ArgumentParser(description="YOLO 框标注工具")
    parser.add_argument("--start", type=int, default=0, help="从第几张开始（0 起）")
    args = parser.parse_args()
    Labeler(args.start).run()


if __name__ == "__main__":
    main()
