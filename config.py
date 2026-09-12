# =========================
# 基础配置文件
# =========================

CAMERA_INDEX = 0

USE_SERIAL = False
SERIAL_PORT = "COM3"
BAUD_RATE = 115200

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

CENTER_TOLERANCE = 80

# YOLO 模型路径
YOLO_MODEL_PATH = "model/shuttlecock.pt"

# YOLO 置信度阈值
# 太低：容易误识别
# 太高：容易漏识别
CONFIDENCE_THRESHOLD = 0.6

# 判断已经靠近羽毛球的面积阈值
# YOLO 框面积超过这个数，就启动捡球机构
TARGET_AREA_NEAR = 45000

COLLECT_TIME = 2.0

SHOW_MASK_WINDOW = False