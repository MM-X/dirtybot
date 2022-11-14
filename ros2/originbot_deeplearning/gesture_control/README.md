# 功能介绍

gesture_control package功能为通过手势控制机器人运动。

订阅智能结果ai_msgs，运行策略，确定控制hand并根据手势控制机器人运动。

通过发布消息直接控制机器人旋转和平移运动。

支持的控制手势和手势功能定义如下表格：

| 手势名称              | 功能定义 | 手势动作举例                                     |
| --------------------- | -------- | ------------------------------------------------ |
| 666手势/Awesome       | 前进     | ![image-awesome](images/image-awesome.png)       |
| yeah/Victory          | 后退     | ![image-palm](images/image-victory.png)          |
| 大拇指向右/ThumbRight | 右转     | ![image-thumbright](images/image-thumbright.png) |
| 大拇指向左/ThumbLeft  | 左转     | ![image-thumbleft](images/image-thumbleft.png)   |
| OK/Okay               | 唤醒     | ![image-ok.png](images/image-ok.png)             |
| 手掌/Palm             | 重置     | ![image-palm](images/image-palm.png)             |

其中点赞、yeah、大拇指向右和大拇指向左4种手势用于控制机器人前后或旋转运动。OK手势用于手势控制唤醒功能，手掌手势用于重置控制功能。

## 唤醒手势

唤醒手势用于唤醒通过手势控制机器人的功能。

当启用唤醒手势时，只有做了唤醒手势的hand才能通过4种控制手势控制机器人。一般用于人较多，环境复杂的场景，通过启用唤醒手势避免误触发手势控制功能。

当未启用唤醒手势时，直接通过上述4种控制手势触发控制机器人功能。

## 重置手势

重置手势用于重置手势控制机器人的功能。只有当启用了唤醒手势时，重置手势才有效。

当识别到控制手作出重置手势时，重置手势控制功能，重新选择控制手。

## 控制手（controler）的选择

当启用唤醒手势时，做唤醒手势的hand会作为controler，只有这个hand做手势才能控制机器人运动。

当未启用唤醒手势时，选择做了上述4种控制手势的hand作为controler。

选择控制手时，如果有多个hand同时做手势，选择hand检测框宽度最大的hand作为controler。

已有controler的情况下，其他hand做唤醒手势或者控制手势都无效。

只有当作为controler的hand消失或者做出重置手势时，才会重新寻找新的controler。连续track_serial_lost_num_thr（默认100）帧未检测到hand，判断hand（controler）消失。

## 控制策略

已找到作为controler的hand后，对于每一帧输入的智能结果处理策略如下：

当前帧中无此控制手，停止机器人运动。

当前帧中有此控制手，判断此hand有无控制手势，如无，停止机器人运动，如有，控制机器人做对应的运动。

# 编译

## 依赖库

ros package：

\- ai_msgs

ai_msgs为自定义的消息格式，用于算法模型推理后，发布推理结果，ai_msgs pkg定义在hobot_msgs中。

## 开发环境

\- 编程语言: C/C++

\- 开发平台: X3/X86

\- 系统版本：Ubuntu 20.0.4

\- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## **编译**

 支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### **Ubuntu板端编译**

1. 编译环境确认 
   - 板端已安装X3 Ubuntu系统。
   - 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
   - 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`
2. 编译

编译命令：`colcon build --packages-select gesture_control`

### Docker交叉编译

1. 编译环境确认

   - 在docker中编译，并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2. 编译

   - 编译命令：

```
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

colcon build --packages-select gesture_control \
   --merge-install \
   --cmake-force-configure \
   --cmake-args \
   --no-warn-unused-cli \
   -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
```

## 注意事项

# 使用介绍

## 依赖

- mipi_cam package：发布图片msg
- hobot_codec package：jpeg图片编码&发布
- mono2d_body_detection package：发布人体、人头、人脸、人手框感知msg
- hand_lmk_detection package：发布人手关键点感知msg
- hand_gesture_detection package：发布手势识别结果msg
- websocket package：渲染图片和ai感知msg

## 参数

| 参数名                    | 类型        | 解释                                           | 是否必须 | 支持的配置                                                                                              | 默认值                        | 是否支持运行时动态配置 |
| ------------------------- | ----------- | ---------------------------------------------- | -------- | ------------------------------------------------------------------------------------------------------- | ----------------------------- | ---------------------- |
| track_serial_lost_num_thr | int         | 目标连续消失帧数阈值。超过此阈值认为目标消失。 | 否       | 无限制                                                                                                  | 100                           | 是                     |
| activate_wakeup_gesture   | int         | 是否启用唤醒手势。                             | 否       | 0/1。0：不启用，1：启用。                                                                               | 0                             | 是                     |
| move_step                 | float       | 平移运动的步长，单位米。                       | 否       | 无限制                                                                                                  | 0.1                           | 是                     |
| rotate_step               | float       | 旋转运动的步长，单位弧度。                     | 否       | 无限制                                                                                                  | 0.5                           | 是                     |
| twist_pub_topic_name      | std::string | 发布Twist类型的运动控制消息的topic名           | 否       | 根据实际部署环境配置。一般机器人订阅的topic为/cmd_vel，ROS2 turtlesim示例订阅的topic为turtle1/cmd_vel。 | /cmd_vel                      | 否                     |
| ai_msg_sub_topic_name     | std::string | 订阅包含手势识别结果的AI消息的topic名          | 否       | 根据实际部署环境配置                                                                                    | /hobot_hand_gesture_detection | 否                     |

## 运行

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行：

### **Ubuntu**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型，根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/mono2d_body_detection/config/ .
cp -r install/lib/hand_lmk_detection/config/ .
cp -r install/lib/hand_gesture_detection/config/ .

ros2 launch install/share/gesture_control/launch/hobot_gesture_control.launch.py
```

### **Linux**

```
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# config中为示例使用的模型，根据实际安装路径进行拷贝
cp -r install/lib/mono2d_body_detection/config/ .
cp -r install/lib/hand_lmk_detection/config/ .
cp -r install/lib/hand_gesture_detection/config/ .

# 启动图片发布pkg
./install/lib/mipi_cam/mipi_cam --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=544 -p io_method:=shared_mem --log-level error &
# 启动jpeg图片编码&发布pkg
./install/lib/hobot_codec/hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg --ros-args --log-level error &
# 启动单目rgb人体、人头、人脸、人手框和人体关键点检测pkg
./install/lib/mono2d_body_detection/mono2d_body_detection --ros-args --log-level error &
# 启动人手关键点检测pkg
./install/lib/hand_lmk_detection/hand_lmk_detection --ros-args --log-level error &
# 启动web展示pkg
./install/lib/websocket/websocket --ros-args -p image_topic:=/image_jpeg -p image_type:=mjpeg -p smart_topic:=/hobot_hand_gesture_detection --log-level error &

# 启动手势识别pkg
./install/lib/hand_gesture_detection/hand_gesture_detection --log-level error &

# 启动手势交互pkg
./install/lib/gesture_control/gesture_control
```

## 注意事项

1. 板端使用launch启动，需要安装依赖，安装命令：`pip3 install lark-parser`。设备上只需要配置一次，断电重启不需要重新配置。
2. 启动小车运动pkg，需要配置驱动：`cp install/lib/xrrobot/config/58-xrdev.rules /etc/udev/rules.d/`，拷贝后重启X3开发板。设备上只需要配置一次，断电重启不需要重新配置。
3. 第一次运行web展示需要启动webserver服务，运行方法为:

- cd 到websocket的部署路径下：`cd install/lib/websocket/webservice/`（如果是板端编译（无--merge-install编译选项）执行命令为`cd install/websocket/lib/websocket/webservice`）
- 启动nginx：`chmod +x ./sbin/nginx && ./sbin/nginx -p .`
- 设备重启需要重新配置。

# 结果分析

## X3结果展示

```

[gesture_control-7] [WARN] [1652965757.145607222] [GestureControlEngine]: Gesture contrl start!, track_id: 2, frame_ts_ms: 3698315325, tracking_sta(0:INITING, 1:TRACKING, 2:LOST): 1, gesture: 11
[gesture_control-7] [WARN] [1652965757.159500951] [GestureControlEngine]: frame_ts_ms: 3698315358, track_id: 2, tracking_sta: 1, gesture: 14
[gesture_control-7] [WARN] [1652965757.159660358] [GestureControlEngine]: do move, direction: 0, step: 0.500000
[gesture_control-7] [WARN] [1652965757.211420964] [GestureControlEngine]: frame_ts_ms: 3698315425, track_id: 2, tracking_sta: 1, gesture: 14
[gesture_control-7] [WARN] [1652965757.211624899] [GestureControlEngine]: do move, direction: 0, step: 0.500000
[gesture_control-7] [WARN] [1652965757.232051230] [GestureControlEngine]: frame_ts_ms: 3698315457, track_id: 2, tracking_sta: 1, gesture: 14
[gesture_control-7] [WARN] [1652965757.232207513] [GestureControlEngine]: do move, direction: 0, step: 0.500000
[gesture_control-7] [WARN] [1652965757.595528850] [GestureControlEngine]: frame_ts_ms 3698315655, track id: 2 recved reset gesture: 5
[gesture_control-7] [WARN] [1652965757.595700337] [GestureControlEngine]: cancel move
```

以上log截取了部分通过手势控制小车运动的处理结果。由于launch文件中配置启用了手势激活功能，在时间戳frame_ts_ms: 3698315325的帧中，通过OK手势（gesture: 11）激活了手势控制功能，从时间戳frame_ts_ms: 3698315358开始通过666手势（gesture: 14）控制小车以0.5m/s的速度前进运动（do move, direction: 0, step: 0.500000）。在时间戳frame_ts_ms 3698315655的帧中，通过手掌手势（gesture: 5）重置了小车运动控制功能，同时使小车停止运动（cancel move）。

## web效果展示



# 常见问题
1、Ubuntu下运行启动命令报错`-bash: ros2: command not found`

当前终端未设置ROS2环境，执行命令配置环境：

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
```

在当前终端执行ros2命令确认当前终端环境是否生效：

```
# ros2
usage: ros2 [-h] Call `ros2 <command> -h` for more detailed usage. ...

ros2 is an extensible command-line tool for ROS 2.

optional arguments:
  -h, --help            show this help message and exit
```

如果输出以上信息，说明ros2环境配置成功。

注意！对于每个新打开的终端，都需要重新设置ROS2环境。

2、做出控制手势，无运动控制消息发布

2.1 检查是否识别到手势

做出控制手势后，查看输出log中“tracking_sta”关键字值是否为1，同时gesture值是否大于0，否则按照“功能介绍”部分手势动作举例确认手势是否标准。

3、终端无log信息输出

3.1 确认launch文件中的node是否都启动成功

重新开启一个终端（仅对Ubuntu系统有效），执行top命令查看launch文件中的node进程是否都在运行，否则使用ros2 run命令单独启动相关node确认启动失败原因。

3.2 查看每个node是否都有发布msg

根据launch文件中每个node配置的发布和订阅的topic名，使用ros2 topic echo（仅对Ubuntu系统有效）命令显示每个topic是否有消息发布，如果无，再确认没有发布的原因。

注意！如果运行ros2 topic命令失败，执行命令安装依赖：`pip3 install netifaces`
