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

# =========================
# 比例控制（方案 3）
# =========================

# 电机速度范围（0~255，对应 Arduino PWM analogWrite）
MAX_SPEED = 200
MIN_SPEED = 80

# P 控制：角速度/差速与水平偏差成正比
# 偏差 1 像素 -> 左右轮速度差 KP 像素单位，按画面半宽 640 归一化
# 差速 = KP * (offset / frame_half_width) * MAX_SPEED
KP = 2.0

# D 控制：抑制振荡，用水平偏差的变化率，0 表示关闭
KD = 0.5

# 目标进入死区内（偏差小于 CENTER_TOLERANCE）时直行的前进速度
FORWARD_SPEED = MAX_SPEED

# 用面积反推距离的增益：distance ≈ DIST_PER_SQRT_AREA / sqrt(area)
# 框面积 45000 时距离约为 DIST_PER_SQRT_AREA / 212 ≈ 2.1（单位随意，相对量）
DIST_PER_SQRT_AREA = 450.0

# 距离小于该值就启动捡球（代替固定面积阈值 TARGET_AREA_NEAR）
COLLECT_DISTANCE = 2.0

# YOLO 模型路径
YOLO_MODEL_PATH = "model/shuttlecock.pt"

# YOLO 置信度阈值
# 太低：容易误识别
# 太高：容易漏识别
CONFIDENCE_THRESHOLD = 0.6

# =========================
# 目标跟踪（方案 4）
# =========================

# 是否启用 ByteTrack 跟踪（model.track），False 退回单帧检测
USE_TRACKER = True

# EMA 平滑系数：0~1，越小越平滑但延迟越大，1 等于不平滑
EMA_ALPHA = 0.4

# 连续丢失多少帧才判定真丢失（期间按最后已知方向盲走）
MISS_FRAME_LIMIT = 10

# 跟踪目标 ID 锁定：一旦锁定某个 ID 就只跟它，None 表示跟分数最高的
# （多球场景下防止频繁换目标；单球场景保持 None 即可）
LOCK_TARGET_ID = None

# 判断已经靠近羽毛球的面积阈值
# YOLO 框面积超过这个数，就启动捡球机构
TARGET_AREA_NEAR = 45000

COLLECT_TIME = 2.0

SHOW_MASK_WINDOW = False