# 羽毛球自动捡球串球机器人

[English](README.en.md) | 中文

基于 YOLO 视觉识别与 Arduino 差速底盘的自动捡羽毛球机器人。机器人自主寻找场地上的羽毛球，驶近后启动捡球机构收球，捡完一颗自动寻找下一颗，支持多球连续作业。

## 功能特性

| 模块 | 说明 |
|------|------|
| 目标识别 | YOLOv8 检测羽毛球，ByteTrack 多目标跟踪，稳定锁定目标 ID |
| 轨迹平滑 | 匀速模型 Kalman 滤波平滑目标位置，漏检期间外推轨迹（最长 1 秒） |
| 行驶控制 | PD 比例差速转向，死区内直行，低速自动补偿防电机堵转 |
| 距离估计 | 支持像素面积估距（默认）与单应变换地面投影（米制，需标定）两种模式 |
| 多球调度 | 捡完一颗自动切换到视野内下一颗，无需重新搜索 |
| 捡球确认 | 视觉确认球是否被捡走，失败就地重试，重试超限进入冷却防止卡死 |
| 安全退出 | Ctrl+C 或按 Q 退出时自动停车、关闭捡球机构、释放资源 |
| 串口看门狗 | 固件侧 500ms 无指令自动停车，捡球期间发心跳保活 |

## 项目结构

```text
automatic_shuttlecock_pickup
│
├── main.py                  # 主程序：主循环、动作执行、调试画面
├── config.py                # 全部参数集中配置（含各方案开关）
├── camera.py                # 相机：后台线程读帧，只保留最新帧
├── detector.py              # YOLO 检测 + ByteTrack 跟踪 + Kalman 平滑
├── kalman_filter.py         # 匀速模型 Kalman 滤波器
├── decision.py              # 决策层：PD 控制、捡球确认、多球调度
├── ground_projection.py     # 单应变换：图像坐标 -> 地面坐标（米）
├── robot_controller.py      # 串口指令封装
├── arduino_controller.ino   # Arduino 固件：PWM 差速 + 看门狗
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

## 工作流程

1. **识别**：YOLO 检测画面中的羽毛球，ByteTrack 分配跟踪 ID，Kalman 平滑位置
2. **追踪**：PD 控制计算左右轮差速，转向对准目标后直行接近
3. **捡球**：目标距离小于阈值（或地面模式下小于 0.25 米）时停车，启动捡球机构 2 秒
4. **确认**：捡球期间统计目标位置球是否仍可见，判断捡球是否成功
5. **换目标**：捡球成功后从当前画面挑选下一颗球直接驶向；失败则就地重试，重试超限冷却 10 秒跳过该球
6. **搜索**：视野内没有球时按最后看到的方向原地旋转搜索

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

## 注意事项

- `MIN_SPEED = 80` 目前未参与控制，实际生效的低速补偿值是 `decision.py` 中硬编码的 40，实车调试以该值为准
- 纯白色检测方案已弃用，白纸、白墙、衣服容易误判，最终方案以 YOLO 模型为准
- 地面标定时 `SQUARE_SIZE` 需按实际打印的棋盘格尺寸修改
