# JetBot 视觉巡线机器人项目

基于 NVIDIA Jetson Nano 和 ROS 的智能视觉巡线机器人，支持动态 PID 调参和 PC 端远程监控。

## 项目特点

- 🚗 实时摄像头图像处理（白底黑线识别）
- 🎮 ROS 分布式通信（PC 端远程监控）
- 📈 动态 PID 调参（`rqt_reconfigure` 支持）
- 🔧 兼容 Adafruit Motor HAT 和 Jetson Nano 4GB

## 硬件需求
| 组件               | 型号/规格                  |
|--------------------|----------------------------|
| 主控模块            | NVIDIA Jetson Nano 4GB     |
| 套件                | JetBot智能车套件            |

## 软件依赖
- **系统**: Ubuntu 18.04 LTS
- **ROS**: Melodic
- **Python**: 2.7python
- **核心库**: OpenCV 4.1.1, Adafruit_MotorHAT, rospy

## 使用方法
1.启动巡线程序：
    roslaunch jetbot_line_follower line_follower.launch
2.PC 端监控（还需要在主机或者虚拟机端配置）：
    查看摄像头图像：
    rosrun image_view image_view image:=/camera/image_raw
    动态调整 PID 参数：
    rosrun rqt_reconfigure rqt_reconfigure
## 项目结构
├── scripts/
│   ├── camera_publisher.py   # 摄像头驱动节点
│   ├── line_detector.py      # 图像处理节点
│   └── pid_controller.py     # PID 控制节点
├── launch/
│   └── line_follower.launch  # ROS 启动文件
├── cfg/
│   └── PID.cfg               # 动态调参配置文件
└── README.md
