"""
实机相机采帧工具。

把机器人（或模拟上车机位）的 USB 相机对准场地，用本脚本采集训练图片。
采集要点：
  1. 相机安装方式必须和上车后完全一致（同支架高度、同俯角），否则白采
  2. 球距离尽量覆盖 0.3 米 ~ 5 米，多角度、多位置摆放
  3. 每张图至少含 1 颗球，可一次拍多颗（标注工具支持一图多框）
  4. 尽量覆盖不同光照（白天/灯光/阴影）

用法：
    python capture_dataset.py                 # 采 300 张
    python capture_dataset.py --total 500     # 自定义张数
    python capture_dataset.py --interval 1.0  # 每张间隔 1 秒
    python capture_dataset.py --prefix night  # 文件名前缀（区分光照场景）

产出：
    dataset/images/*.jpg
    按空格拍摄一张，Q 结束（自动保存进度，可中断续拍）。
"""

import argparse
import time
from pathlib import Path

import cv2
from config import CAMERA_INDEX, FRAME_WIDTH, FRAME_HEIGHT

DATASET_DIR = Path(__file__).parent / "dataset"
IMAGES_DIR = DATASET_DIR / "images"


def next_index(images_dir: Path) -> int:
    """返回下一个可用序号（断点续拍）。"""
    existing = list(images_dir.glob("*.jpg"))
    return len(existing)


def main():
    parser = argparse.ArgumentParser(description="实机相机采帧工具")
    parser.add_argument("--total", type=int, default=300, help="目标总张数")
    parser.add_argument("--interval", type=float, default=0.5, help="两张图最小间隔（秒）")
    parser.add_argument("--prefix", default="", help="文件名前缀，区分不同拍摄场景")
    args = parser.parse_args()

    IMAGES_DIR.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError("无法打开相机，请检查 CAMERA_INDEX")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    saved = next_index(IMAGES_DIR)
    target = saved + args.total
    print(f"已拍 {saved} 张，本次目标 {target} 张（还需 {args.total} 张）")
    print("空格：拍摄一张 | Q：结束")

    last_save = 0.0
    while saved < target:
        ret, frame = cap.read()
        if not ret:
            print("读帧失败，1 秒后重试...")
            time.sleep(1.0)
            continue

        cv2.imshow("Capture", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        if key == ord(" ") and saved < target:
            now = time.time()
            if now - last_save < args.interval:
                continue
            prefix = f"{args.prefix}-" if args.prefix else ""
            out = IMAGES_DIR / f"{prefix}{saved:05d}.jpg"
            cv2.imwrite(str(out), frame)
            saved += 1
            last_save = now
            print(f"已保存 {out.name}（{saved}/{target}）")
            # 保存后短暂暂停，让人调整球的摆放/机位
            time.sleep(args.interval)

    cap.release()
    cv2.destroyAllWindows()
    print(f"完成：dataset/images 共 {len(list(IMAGES_DIR.glob('*.jpg')))} 张图片")
    print("下一步：python label_tool.py 进行标注")


if __name__ == "__main__":
    main()
