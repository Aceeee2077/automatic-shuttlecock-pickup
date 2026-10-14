"""
YOLO 模型实机视角微调。

原 model/shuttlecock.pt 用手机视角数据集训练，上车后是固定俯视机位，
用 capture_dataset.py 采的实机视角图片做增量微调：
  - 基于原权重继续训练（保留手机视角泛化能力）
  - 实机数据集 90% 训练 + 10% 验证
  - CPU 可跑（YOLO11n 约 300 张图 × 50 轮，几十分钟）

用法：
    python finetune_model.py                # 默认 50 轮
    python finetune_model.py --epochs 80    # 自定义轮数
    python finetune_model.py --imgsz 640    # 输入分辨率
    python finetune_model.py --data dataset/robot.yaml   # 显式指定

产出：
    runs/detect/finetune/weights/best.pt    # 验证集上最优权重
训练完成后把 best.pt 复制覆盖 model/shuttlecock.pt 即可上车。

前置：
    1. python capture_dataset.py   采图（至少 200 张）
    2. python label_tool.py        标注
"""

import argparse
import random
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).parent
BASE_MODEL = ROOT / "model" / "shuttlecock.pt"
IMAGES_DIR = ROOT / "dataset" / "images"
LABELS_DIR = ROOT / "dataset" / "labels"
OUT_DIR = ROOT / "runs" / "detect" / "finetune"


def check_dataset():
    images = sorted(IMAGES_DIR.glob("*.jpg"))
    if len(images) < 50:
        print(f"图片太少（{len(images)} 张，建议 >= 200），先运行 capture_dataset.py")
        sys.exit(1)

    labeled = 0
    for img in images:
        lf = LABELS_DIR / (img.stem + ".txt")
        if lf.exists() and lf.read_text().strip():
            labeled += 1
    if labeled == 0:
        print("没有已标注的图片，先运行 label_tool.py")
        sys.exit(1)

    print(f"图片 {len(images)} 张，其中已标注 {labeled} 张")
    if labeled < 0.8 * len(images):
        print("注意：标注率不足 80%，未标注图会被丢弃，请确认已标完想用的图")


def split_dataset(ratio=0.9, seed=42):
    """把已标注图按 9:1 切分 train/val，生成 YOLO 数据集目录与 data.yaml。

    复用原 YOLO 单类数据集结构：
        dataset/
          train/images  train/labels
          val/images    val/labels
          robot.yaml
    """
    images = []
    for img in sorted(IMAGES_DIR.glob("*.jpg")):
        lf = LABELS_DIR / (img.stem + ".txt")
        if lf.exists() and lf.read_text().strip():
            images.append((img, lf))

    random.seed(seed)
    random.shuffle(images)

    n_val = max(1, int(len(images) * (1.0 - ratio)))
    val_set = images[:n_val]
    train_set = images[n_val:]

    if len(train_set) == 0:
        print("标注图太少，切分后训练集为空")
        sys.exit(1)

    for split, items in (("train", train_set), ("val", val_set)):
        (ROOT / "dataset" / split / "images").mkdir(parents=True, exist_ok=True)
        (ROOT / "dataset" / split / "labels").mkdir(parents=True, exist_ok=True)
        # 清空旧文件，避免残留
        for old in (ROOT / "dataset" / split / "images").glob("*.jpg"):
            old.unlink()
        for old in (ROOT / "dataset" / split / "labels").glob("*.txt"):
            old.unlink()
        for img, lf in items:
            shutil.copy2(img, ROOT / "dataset" / split / "images" / img.name)
            shutil.copy2(lf, ROOT / "dataset" / split / "labels" / lf.name)

    yaml_path = ROOT / "dataset" / "robot.yaml"
    yaml_path.write_text(
        "path: " + str(ROOT / "dataset").replace("\\", "/") + "\n"
        "train: train/images\n"
        "val: val/images\n"
        "names:\n"
        "  0: shuttlecock\n"
    )

    print(f"切分完成：train {len(train_set)} 张 / val {len(val_set)} 张")
    print(f"数据集描述：{yaml_path}")
    return yaml_path


def main():
    parser = argparse.ArgumentParser(description="实机视角 YOLO 微调")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--data", default=None, help="data.yaml 路径（默认自动生成）")
    parser.add_argument("--device", default="cpu", help="cpu / 0 / 0,1")
    args = parser.parse_args()

    if not BASE_MODEL.exists():
        print(f"找不到基础模型 {BASE_MODEL}")
        sys.exit(1)

    check_dataset()
    data_yaml = args.data or str(split_dataset())

    from ultralytics import YOLO

    # 增量微调：继承原模型权重与类别（单类 shuttlecock）
    model = YOLO(str(BASE_MODEL))
    results = model.train(
        data=data_yaml,
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=args.device,
        project=str(ROOT / "runs" / "detect"),
        name="finetune",
        exist_ok=True,
        patience=15,          # 早停：15 轮无提升即停
        lr0=0.005,            # 增量训练用小学习率
        warmup_epochs=2.0,
        val=True,
        plots=True,
        save=True,
        verbose=True,
    )

    best = OUT_DIR / "weights" / "best.pt"
    if best.exists():
        print(f"\n训练完成，最优权重：{best}")
        print(f"上车步骤：复制 best.pt 覆盖 {BASE_MODEL}")
        print(f"命令（PowerShell）：Copy-Item '{best}' '{BASE_MODEL}' -Force")
    else:
        print("\n训练未产出 best.pt，请检查输出目录")


if __name__ == "__main__":
    main()
