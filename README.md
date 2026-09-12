# Automatic Shuttlecock Pickup Machine

这是一个自动捡羽毛球机器人的软件 MVP 项目。

当前版本功能：

1. 使用摄像头读取画面
2. 使用 OpenCV 白色物体检测作为第一版识别方案
3. 判断目标在画面左侧、右侧或中间
4. 根据判断结果输出移动指令
5. 支持串口发送指令给 Arduino / ESP32
6. 支持后续替换成 YOLO 羽毛球识别模型

## 安装依赖

```bash
python -m pip install -r requirements.txt
```

## 运行

```bash
python main.py
```

## 退出

按键盘 `q` 退出程序。

## 文件结构

```text
automatic_shuttlecock_pickup_machine
│
├── main.py
├── config.py
├── camera.py
├── detector.py
├── decision.py
├── robot_controller.py
├── requirements.txt
├── README.md
│
└── model
       └── 放 YOLO 模型，例如 shuttlecock.pt
```

## 注意

当前 `detector.py` 使用的是传统 OpenCV 白色区域检测，不是最终方案。

最终产品建议使用 YOLO 模型识别羽毛球，否则白纸、白墙、衣服、反光等可能被误判。
