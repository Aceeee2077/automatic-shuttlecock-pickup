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

# =========================
# Kalman 滤波（方案 8）
# =========================

# 启用匀速模型 Kalman 替代 EMA 平滑位置（漏检期间可外推运动轨迹）
# False 时退回原 EMA 全量平滑
USE_KALMAN = True

# 过程噪声：越大越相信新测量（响应快、平滑弱）
KALMAN_POS_NOISE = 4.0
KALMAN_VEL_NOISE = 1.0

# 测量噪声（像素方差）：越大越平滑但延迟越大
KALMAN_MEAS_NOISE = 9.0

# =========================
# 地面投影（方案 9）
# =========================

# 启用后按地面方位角/距离闭环控制（需先运行 camera_calibration.py
# 和 calibrate_homography.py 两个标定脚本）；False 退回像素模式
USE_GROUND_PROJECTION = False

# calibrate_homography.py 生成的单应矩阵
HOMOGRAPHY_PATH = "homography.npz"

# camera_calibration.py 生成的内参（可选，用于去畸变）
CALIBRATION_PATH = "calibration.npz"

# 方位角死区（rad，约 7 度）：|bearing| 小于该值时直行
ANGLE_TOLERANCE = 0.12

# 方位角 PD 增益
KP_ANGLE = 1.6
KD_ANGLE = 0.3

# 直行前进速度随距离衰减：fwd = min(FORWARD_SPEED, distance * 该值)
FORWARD_DISTANCE_GAIN = 600.0

# 地面距离小于该值（米）启动捡球（仅地面投影模式生效）
COLLECT_DISTANCE_M = 0.25

# 判断已经靠近羽毛球的面积阈值
# YOLO 框面积超过这个数，就启动捡球机构
TARGET_AREA_NEAR = 45000

COLLECT_TIME = 2.0

SHOW_MASK_WINDOW = False

# =========================
# 多球调度（方案 10）
# =========================

# 捡球期间继续跟踪画面，捡完后直接驶向下一个目标（而不是原地 SEARCH）
MULTI_BALL = True

# 下一个目标与被捡球中心的最小距离（像素），防止把同一个框当成两颗球
NEXT_TARGET_MIN_SEP = 120

# =========================
# 捡球确认（方案 11）
# =========================

# 捡球结束后用视觉确认球已进入机构：
# 捡球期间被捡球原位置附近仍持续看到球，则判定失败并就地重试
COLLECT_CONFIRM = True

# 捡球期间统计"被捡球位置附近仍可见"的帧占比，超过该比例判定失败
COLLECT_CONFIRM_RATIO = 0.5

# 确认半径（像素）：候选框中心落在被捡球中心该半径内视为同一颗球
COLLECT_CONFIRM_RADIUS = 150

# 失败后就地重试的最大次数，超过后冷却一段时间不再触发捡球
MAX_COLLECT_RETRIES = 2

# 重试超限后的捡球冷却时间（秒），期间 COLLECT 动作替换为 SEARCH
COLLECT_RETRY_COOLDOWN = 10.0

# =========================
# 轮速加速度限幅（方案 12）
# =========================

# 限制每秒最大 PWM 增量，平滑加减速，防止急转/急停打滑、丢球
# 按帧率 30fps 估算：单帧增量 ≈ MAX_ACCEL / 30
MAX_ACCEL = 1200.0

# 是否启用加速度限幅，False 退回无限制（立即到达目标速度）
USE_ACCEL_LIMIT = True