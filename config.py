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

# =========================
# 障碍物检测与避障（方案 19）
# =========================

# 启用障碍物检测（需模型含 person/bag/chair 类别或第二个轻量模型）
USE_OBSTACLE_AVOID = False

# 障碍物模型路径；为 None 时复用羽毛球模型（模型需含障碍物类别）
OBSTACLE_MODEL_PATH = None

# 视为障碍物的类别名（COCO 类别名）
OBSTACLE_CLASSES = ("person", "backpack", "handbag", "chair")

# 障碍物检测置信度阈值
OBSTACLE_CONF_THRESHOLD = 0.5

# 障碍物投影距离小于该值（米，地面模式）或框面积超过该值（像素模式
# 近似近距）时避障
OBSTACLE_STOP_DISTANCE_M = 0.5
OBSTACLE_AREA_NEAR = 80000

# 障碍物持续遮挡超时（秒），超过后放弃该目标进入搜索
OBSTACLE_BLOCK_TIMEOUT = 8.0

# =========================
# 电量监测（方案 20）
# =========================

# 启用电池电压监测（需固件支持 BATT 上报）
USE_BATTERY_MONITOR = False

# 低电压报警阈值（mV）
BATTERY_LOW_MV = 10500

# 严重低电压阈值（mV），低于该值禁止启动新任务并强制停车
BATTERY_CRITICAL_MV = 10000

# =========================
# Web 远程监控（方案 22）
# =========================

# 启用 Web 监控服务（浏览器打开 http://<ip>:8000）
USE_WEB_SERVER = False

# 监控服务端口
WEB_SERVER_PORT = 8000

# JPEG 推流质量（1~100）
WEB_JPEG_QUALITY = 70

# =========================
# 三段式接近（方案 26）
# =========================

# 按距离切换控制档位（仅地面投影模式生效），近处低速精调，防撞、对准更准
# 远距（>= STAGED_NEAR_DISTANCE_M）：全速 PD 正常接近
# 近距（<  STAGED_NEAR_DISTANCE_M）：蠕行档，低速 + 更强转向增益，
#   到位后暂停观察 STAGED_SETTLE_TIME 秒确认落位再捡球
USE_STAGED_APPROACH = True

# 近距档距离阈值（米），小于该值进入蠕行
STAGED_NEAR_DISTANCE_M = 1.0

# 蠕行档最大前进速度（PWM），低速保证到位精度
STAGED_NEAR_SPEED = 60

# 蠕行档方位角 PD 增益（低速下需要更猛的转向才能对准）
STAGED_KP_ANGLE = 2.2
STAGED_KD_ANGLE = 0.3

# 到位暂停观察：近距且方位角进死区时，先停车观察确认落位再触发捡球
# 防止投影误差导致没到跟前就开捡（按时间而非帧数，推理耗时不固定）
STAGED_SETTLE_TIME = 1.0

# =========================
# 卡死检测与自动脱困（方案 27）
# =========================

# 启用卡死检测：持续前进但目标距离不缩小 -> 判定卡死并自动脱困
USE_STUCK_DETECT = True

# 判定卡死所需的持续时长（秒），期间距离无实质变化
STUCK_DURATION = 2.0

# 距离变化容差（米）：持续时长内距离缩小少于该值视为"没动"
STUCK_DISTANCE_EPS = 0.15

# 脱困动作参数
STUCK_REVERSE_SECONDS = 0.5    # 后退时长
STUCK_REVERSE_SPEED = 80       # 后退速度（PWM）
STUCK_TURN_SECONDS = 0.6       # 原地转时长（约 30~45 度，视底盘而定）
STUCK_TURN_SPEED = 80          # 原地转速度（PWM）

# 同一目标最大脱困次数，超过后冷却跳过该球（防原地无限循环）
STUCK_MAX_ESCAPES = 3

# 脱困超限后的冷却时间（秒），期间不逼近该目标
STUCK_RETRY_COOLDOWN = 8.0