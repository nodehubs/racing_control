# 功能介绍

该功能包通过同时接收赛道识别节点与障碍物识别节点的消息，控制小车巡线与避障

# 使用方法

## 准备工作

1. 具备真实的机器人或机器人仿真模块，包含运动底盘、相机及RDK套件，并且能够正常运行。
2. 已有ROS底层驱动，机器人可接收“/cmd_vel”指令运动，并根据指令正确运动。

## 安装功能包

**1.安装功能包**

启动机器人后，通过终端SSH或者VNC连接机器人，点击本页面右上方的“一键部署”按钮，复制如下命令在RDK的系统上运行，完成相关Node的安装。

```bash
sudo apt update
sudo apt install -y tros-racing-control
```

**2.运行自动巡线与避障功能**

```shell
source /opt/tros/local_setup.bash

# 仿真
ros2 launch racing_control racing_control_simulation.launch.py

# 实际场景
ros2 launch racing_control racing_control.launch.py
```


# 接口说明

## 话题

### Pub话题

| 名称                          | 消息类型                                                     | 说明                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /cmd_vel    | geometry_msgs/msg/Twist             | 发布小车控制消息                 |

### Sub话题
| 名称                          | 消息类型                                                     | 说明                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| racing_track_center_detection      | geometry_msgs/msg/PointStamped        | 接收赛道中点的位置消息                  |
| racing_obstacle_detection      | ai_msgs/msgs/PerceptionTargets        | 接收检测到的障碍物的位置信息                 |

## 参数

| 参数名                | 类型        | 说明   |
| --------------------- | ----------- | ----------------------------------------------------- |
| pub_control_topic    | string |    发布的控制消息名称，请根据实际发布的话题名称配置，默认值为/cmd_vel |
| avoid_angular_ratio   | float | 避障时的角速度比例，请根据实际情况配置，默认值为1.1 |
| avoid_linear_speed   | float | 避障时的线速度，请根据实际情况配置，默认值为0.25 |
| follow_angular_ratio   | float | 巡线时的角速度比例，请根据实际情况配置，默认值为-1.0 |
| follow_linear_speed   | float | 巡线时的线速度，请根据实际情况配置，默认值为1.5 |
| bottom_threshold   | int | 触发避障功能的坐标阈值（目标底部坐标的y值），请根据实际情况配置，默认值为340 |
