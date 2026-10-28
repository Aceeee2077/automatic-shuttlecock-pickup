# 羽毛球自动捡球串球机器人 (软件部分)

[English](README.en.md) | 中文

基于 YOLO 视觉识别与 Arduino 差速底盘的自动捡羽毛球机器人。机器人自主寻找场地上的羽毛球，驶近后启动捡球机构收球，捡完一颗自动寻找下一颗，支持多球连续作业。

## 功能特性

| 模块 | 说明 |
|------|------|
| 目标识别 | YOLOv8 检测羽毛球，ByteTrack 多目标跟踪，稳定锁定目标 ID |
| 轨迹平滑 | 匀速模型 Kalman 滤波平滑目标位置，漏检期间外推轨迹（最长 1 秒） |
| 行驶控制 | PD 比例差速转向，死区内直行，轮速加速度限幅防打滑，低速自动补偿防电机堵转 |
| 距离估计 | 支持像素面积估距（默认）与单应变换地面投影（米制，需标定）两种模式 |
| 多球调度 | 捡完一颗自动切换到视野内下一颗，无需重新搜索 |
| 捡球确认 | 视觉确认球是否被捡走，失败就地重试，重试超限进入冷却防止卡死 |
| 障碍物避障 | 独立 YOLO 模型检测行人、背包等挡路障碍物，挡路停车等待，持续遮挡超时放弃目标 |
| 电量监测 | 固件每秒上报电池电压，低电压报警节流，严重低电压自动停车，画面与网页同步显示电量 |
| Web 远程监控 | 内置 MJPEG 推流 + 状态接口，浏览器打开 `http://<ip>:8000` 远程查看画面、动作、电量与捡球计数 |
| 三段式接近 | 地面模式下近距（< 1 米）自动切换蠕行档：低速 + 更强转向增益，到位后暂停观察确认落位再捡球，防冲过、对准更准 |
| 卡死检测脱困 | 持续行驶但目标距离不缩小判卡死，自动后退 + 原地转脱困；同目标脱困超限进入冷却跳过该球 |
| 安全退出 | Ctrl+C 或按 Q 退出时自动停车、关闭捡球机构、释放资源 |
| 串口看门狗 | 固件侧 500ms 无指令自动停车，捡球期间发心跳保活 |
| 启动自检 | 启动前校验 `config.py` 数值范围、电量阈值顺序、模型与标定文件；错误直接阻止启动，警告继续运行 |

## 项目结构

```text
automatic_shuttlecock_pickup
│
├── main.py                  # 主程序：主循环、动作执行、调试画面
├── config.py                # 全部参数集中配置（含各方案开关）
├── preflight.py             # 启动自检：参数范围、电量阈值、模型/标定文件
├── camera.py                # 相机：后台线程读帧，只保留最新帧
├── detector.py              # YOLO 检测 + ByteTrack 跟踪 + Kalman 平滑
├── kalman_filter.py         # 匀速模型 Kalman 滤波器
├── decision.py              # 决策层：PD 控制、捡球确认、多球调度、避障
├── ground_projection.py     # 单应变换：图像坐标 -> 地面坐标（米）
├── robot_controller.py      # 串口指令封装 + 后台解析固件上报（电量）
├── arduino_controller.ino   # Arduino 固件：PWM 差速 + 看门狗 + 电量上报
├── web_server.py            # Web 远程监控：MJPEG 推流 + 状态 JSON（可选）
├── camera_calibration.py    # 相机内参标定脚本（可选）
├── calibrate_homography.py  # 地面单应矩阵标定脚本（可选）
├── requirements.txt
└── model/
    └── shuttlecock.pt       # YOLO 羽毛球模型
```

## 安装

```bash
python -m pip install -r requirements.txt
```

依赖：Python 3.8+、ultralytics（YOLO）、OpenCV、pyserial、numpy。

将训练好的 YOLO 模型放到 `model/shuttlecock.pt`。

## 运行

```bash
python main.py
```

- 按 `Q` 退出，或 Ctrl+C 中断（两者都会自动停车）
- 默认不连串口（`USE_SERIAL = False`），只显示画面和控制量，方便先调试视觉
- 实车运行时在 `config.py` 中设置 `USE_SERIAL = True` 和 `SERIAL_PORT`

启动时会先运行 `preflight.py`：数值越界、`BATTERY_CRITICAL_MV` 不低于 `BATTERY_LOW_MV`、模型或地面单应文件缺失等错误会直接阻止启动；未做直行标定、障碍物模型复用羽毛球模型等情况只打印警告。这样不会在配置有误时先打开相机、串口或日志目录。

运行离线单元测试（不需要相机、Arduino 或模型推理）：

```bash
python -m unittest discover -s tests -v
```

## 配置说明

所有参数集中在 `config.py`，常用项：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `USE_SERIAL` | `False` | 是否连接串口，调试视觉时保持 False |
| `SERIAL_PORT` | `"COM3"` | Arduino 串口号 |
| `CONFIDENCE_THRESHOLD` | `0.6` | YOLO 置信度阈值，误检多就调高 |
| `MAX_SPEED` | `200` | 电机 PWM 上限（0~255） |
| `CENTER_TOLERANCE` | `80` | 像素模式转向死区（像素） |
| `USE_TRACKER` | `True` | ByteTrack 跟踪开关 |
| `USE_KALMAN` | `True` | Kalman 滤波开关，False 退回 EMA |
| `USE_GROUND_PROJECTION` | `False` | 地面投影模式开关，需先标定 |
| `MULTI_BALL` | `True` | 多球调度开关 |
| `COLLECT_CONFIRM` | `True` | 捡球视觉确认开关 |
| `MAX_ACCEL` | `1200.0` | 轮速加速度限幅（PWM/秒），防急转打滑 |
| `USE_OBSTACLE_AVOID` | `False` | 障碍物避障开关 |
| `OBSTACLE_MODEL_PATH` | `None` | 障碍物模型路径，None 复用羽毛球模型 |
| `OBSTACLE_CLASSES` | `("person", ...)` | 障碍物类别（行人、背包等） |
| `OBSTACLE_STOP_DISTANCE_M` | `0.5` | 障碍物停车距离（地面模式，米） |
| `OBSTACLE_AREA_NEAR` | `80000` | 障碍物近似阈值（像素模式，面积大 = 近） |
| `OBSTACLE_BLOCK_TIMEOUT` | `8.0` | 持续遮挡超时（秒），超时放弃目标 |
| `USE_BATTERY_MONITOR` | `False` | 电量监测开关 |
| `BATTERY_LOW_MV` | `10500` | 低电压阈值（mV），低于则报警 |
| `BATTERY_CRITICAL_MV` | `10000` | 严重低电压阈值（mV），低于则强制停车 |
| `USE_WEB_SERVER` | `False` | Web 远程监控开关 |
| `WEB_SERVER_PORT` | `8000` | Web 监控端口 |
| `WEB_JPEG_QUALITY` | `70` | Web 推流 JPEG 质量（1~100） |
| `USE_STAGED_APPROACH` | `True` | 三段式接近开关（仅地面模式） |
| `STAGED_NEAR_DISTANCE_M` | `1.0` | 近距蠕行档距离阈值（米） |
| `STAGED_NEAR_SPEED` | `60` | 蠕行档最大前进速度（PWM） |
| `STAGED_KP_ANGLE` / `STAGED_KD_ANGLE` | `2.2 / 0.3` | 蠕行档方位角 PD 增益（低速需更猛转向） |
| `STAGED_SETTLE_TIME` | `1.0` | 到位暂停观察时长（秒），确认落位再捡球 |
| `USE_STUCK_DETECT` | `True` | 卡死检测开关（仅地面模式） |
| `STUCK_DURATION` | `2.0` | 判定卡死所需持续时长（秒） |
| `STUCK_DISTANCE_EPS` | `0.15` | 距离变化容差（米），小于视为没动 |
| `STUCK_REVERSE_SECONDS` / `STUCK_REVERSE_SPEED` | `0.5 / 80` | 脱困后退时长（秒）与速度（PWM） |
| `STUCK_TURN_SECONDS` / `STUCK_TURN_SPEED` | `0.6 / 80` | 脱困原地转时长（秒）与速度（PWM） |
| `STUCK_MAX_ESCAPES` | `3` | 同目标最大脱困次数，超过进入冷却 |
| `STUCK_RETRY_COOLDOWN` | `8.0` | 脱困超限后的冷却时间（秒） |

## 工作流程

1. **识别**：YOLO 检测画面中的羽毛球，ByteTrack 分配跟踪 ID，Kalman 平滑位置
2. **追踪**：PD 控制计算左右轮差速，转向对准目标后直行接近；地面模式下近距（< 1 米）自动切蠕行档（低速 + 更强转向增益），到位后暂停观察 1 秒确认落位再捡球
3. **捡球**：目标距离小于阈值（或地面模式下小于 0.25 米）时停车，启动捡球机构 2 秒
4. **确认**：捡球期间统计目标位置球是否仍可见，判断捡球是否成功
5. **换目标**：捡球成功后从当前画面挑选下一颗球直接驶向；失败则就地重试，重试超限冷却 10 秒跳过该球
6. **搜索**：视野内没有球时按最后看到的方向原地旋转搜索
7. **避障**（可选）：行驶途中检测到行人等障碍物挡在目标与机器人之间时停车等待，持续遮挡超过 8 秒放弃该目标转为搜索
8. **电量保护**（可选）：电池电压低于 10.5V 报警，低于 10V 自动停车并关闭捡球机构
9. **卡死脱困**（仅地面模式）：行驶中持续 2 秒目标距离不缩小判卡死，自动后退 0.5 秒 + 原地转 0.6 秒后重新驶向；同一球脱困超过 3 次进入 8 秒冷却跳过

## 可选：地面投影标定

地面投影模式用真实方位角和米制距离做闭环，控制量与相机安装高度、分辨率解耦，换装机位不用重新调参。启用步骤：

1. **相机内参标定**：打印 9×6 棋盘格（按 `camera_calibration.py` 中 `SQUARE_SIZE` 的实际尺寸），运行 `python camera_calibration.py`，对棋盘格从多个角度拍摄至少 10 张（按空格拍摄，按 Q 结束），生成 `calibration.npz`
2. **单应矩阵标定**：把机器人摆到场地中央，运行 `python calibrate_homography.py`，依次点击画面中 4 个已知地面点（机器人正下方、正前方、右前方、右侧），输入对应实际距离，生成 `homography.npz`
3. 在 `config.py` 中设置 `USE_GROUND_PROJECTION = True`

不标定也能跑：默认像素模式用面积估距，精度稍差但零配置。

## 串口协议

波特率 115200，指令以换行符结尾。

| 指令 | 说明 |
|------|------|
| `SPEED <L> <R>` | 左右轮 PWM 速度，0~255 |
| `STOP` | 停车 |
| `COLLECT_ON` / `COLLECT_OFF` | 捡球电机开/关 |
| `TURN_LEFT` / `TURN_RIGHT` | 原地转向（搜索用） |
| `PING` | 心跳，仅刷新看门狗 |

固件侧看门狗 500ms 内没收到任何指令自动停车并关闭捡球电机；上电后未收到指令不会触发。

固件每秒上报一次电池电压：`BATT <电压毫秒值>`，格式为 `BATT 11200`（11.2V）。

## 注意事项

- `MIN_SPEED = 80` 目前未参与控制，实际生效的低速补偿值是 `decision.py` 中硬编码的 40，实车调试以该值为准
- 纯白色检测方案已弃用，白纸、白墙、衣服容易误判，最终方案以 YOLO 模型为准
- 地面标定时 `SQUARE_SIZE` 需按实际打印的棋盘格尺寸修改
- 电量监测需在电池与 Arduino A0 之间接分压电阻（10k + 3.3k，分压比 4.03），并按实际电阻值修改固件中的 `VOLTAGE_DIVIDER_RATIO`；A0 输入电压不得超过 5V
- 避障默认复用羽毛球模型（不含行人类别），实际使用需训练一个含 person/backpack 等类别的独立模型，并在 `OBSTACLE_MODEL_PATH` 指定
- Web 监控仅用于局域网观察，无鉴权，不要暴露到公网
