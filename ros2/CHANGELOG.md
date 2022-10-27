# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v1.0.0] - 2022-10-09
### 优化
- 优化网站内容，完成首发套件各项功能测试

## [v0.6/v1.0-rc1] - 2022-9-23
### 新增
- 新增originbot_demo文件夹，放置基础功能的编程示例

## [v0.5] - 2022-9-20
### 新增
- 新增[originbot_desktop](https://gitee.com/guyuehome/originbot_desktop)仓库，放置电脑端运行的功能包，避免机器人端的代码编译错误

### 优化
- 优化坐标系结构，新增base_footprint坐标系，保持与URDF模型一致
- 优化手柄遥控运动机器人功能
- 优化AI视觉跟踪应用的效果
- 优化AI视觉巡线功能，增加训练使用的数据集和训练好的模型
- 优化Ctrl+C关闭小车ROS驱动时，机器人自动停止运动
- [originbot_desktop](https://gitee.com/guyuehome/originbot_desktop)：优化上位机可视化显示过程，添加常用功能的rviz2配置脚本 


## [v0.4] - 2022-9-3

### 优化
- 修复机器人底盘use_imu参数没有调用的问题
- 调整人体跟踪与手势识别的旋转速度
- 自动停车功能默认关闭
- 优化文件夹结构
- 删除电脑端运行使用的功能包，避免在机器人端编译出错


## [v0.3] - 2022-8-31
### 新增
- OriginBot的URDF可视化模型
- OriginBot仿真模型（模型简化，提高gazebo运行效率）
- OriginBot仿真环境、相机仿真、雷达仿真

### 优化
- 机器人各坐标系通过URDF进行描述：base_link、laser_link、imu_link、camera_link
- 删除暂时用不到的二位码识别包


## [v0.2] - 2022-8-25
### 新增
- 基于深度学习的视觉巡线功能

### 优化
- 不同传感器的启动脚本都集成在originbot.launch.py中，通过参数来设置
- 调试视觉巡线功能，基于HSV模型识别制定区域内的某种颜色中心，从而转化成控制指令
- 旭日派中ROS启动成功时，蜂鸣器响0.5s


## [v0.1] - 2022-8-19
### 新增
- 提供OriginBot核心功能的驱动节点
    - 支持旭日派与控制器之间的串口驱动
    - 支持里程计话题的发布与TF坐标系维护
    - 支持IMU话题的发布
    - 支持速度控制指令的订阅与运动控制，可选是否启动自动停车功能
- 提供OriginBot常用应用功能包
    - 支持Cartographer SLAM地图构建功能
    - 支持Navigation2定位与导航功能
    - 支持手势控制功能
    - 支持人体跟踪功能

### 问题
- 摄像头驱动节点的标定话题待完善
- 手柄遥控运动待测试
- 二维码识别功能待测试
- 视觉巡线功能待测试
