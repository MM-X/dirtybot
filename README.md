# dirtybot
**脏宝(dirtybot, diety bea)—低成本桌面助理机器人**

<iframe src="//player.bilibili.com/player.html?aid=816914493&bvid=BV1wG4y1h7TZ&cid=873782613&page=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>

# 一、设计目标

1. **桌面级的机器人，要够小**
2. **需要有执行器——轮子、推铲、扫地机**
3. **可玩性和可扩展性要强**
4. **智能性够强，成本要够低**

## 预计可实现的功能：

### 按具体功能划分：

- **自主清理垃圾**——将桌面大的垃圾推到地面，小的垃圾（和尘土）收集进入垃圾储藏室
- **语音控制**、手机**App控制**、键盘控制、手势控制、人体跟踪控制
- 人体**跟踪**、猫狗跟踪、**语音交互**（到杯子那里去，到垃圾桶那里去，把杯子推回来）

### 按涉及领域划分：

- **语音交互：**语音唤醒、语音开关机、语音控制车辆运动、语音调参、语音启停进程（开启关闭人体跟踪控制）、被识别物体语音播报
- **感知：**目标检测与分割、根据自车位置推测目标的平面位置和尺寸（结合cam和lidar）
- **定位建图：**slam建图和更新（包括正障碍-杯子， 结合tof建立负障碍-桌面边缘）
- **规划控制：**导航规划，路径规划，跟踪控制

# 二、 硬件组成

![dirtybot_model1](docs/pic/dirtybot_model1.png)

![dirtybot_model2](docs/pic/dirtybot_model2.png)

|     名称     | 型号 | 数量 | 价格 |                                                              |      |
| :----------: | :--: | :--: | :--: | :----------------------------------------------------------: | :--: |
|    开发板    | X3派 |  1   |      | <img src="docs/pic/-16668862277965.png" alt="img" style="zoom:50%;" /> |      |
|    摄像头    |      |  1   |      | <img src="docs/pic/-16668862366527.png" alt="img" style="zoom:50%;" /> |      |
|   激光雷达   |      |  1   |      | <img src="docs/pic/-16668862397339.png" alt="img" style="zoom: 67%;" /> |      |
|     电机     |      |  2   |      | <img src="docs/pic/-166688626961711.png" alt="img" style="zoom:50%;" /> |      |
|     MCU      | Nano |  1   |      | <img src="docs/pic/-166688628094913.png" alt="img" style="zoom:67%;" /> |      |
|   语音模块   |      |  1   |      | <img src="docs/pic/-166688628359415.png" alt="img" style="zoom:67%;" /> |      |
|   电机驱动   |      |  2   |      | <img src="docs/pic/-166688628568317.png" alt="img" style="zoom:67%;" /> |      |
| 激光测距模块 |      |  2   |      | <img src="https://horizonrobotics.feishu.cn/space/api/box/stream/download/asynccode/?code=MzExYmExMzk5YjY5ZTNlNzg3MTI3YmVmNWRhMWE5OWFfTHhiOFZkWTBUbWVRY201YnVvaXhXRDh0Y1U2V0VFR0JfVG9rZW46Ym94Y25vZGNpRWp5UWM1STMwd2hjRmVGWU5lXzE2NjY4ODYxOTQ6MTY2Njg4OTc5NF9WNA" alt="img" style="zoom:80%;" /> |      |
|     IMU      |      |  1   |      | <img src="https://horizonrobotics.feishu.cn/space/api/box/stream/download/asynccode/?code=NTIxZTQxOTA0ZTY4MzdkYjAxZDNiYTA0MDE1NGM4NzRfMW9SZ3hOWVdzcTZwbTY2UHhRcnVGcTBvNEtoRndiMzVfVG9rZW46Ym94Y25iNHVQVEdvMEt1UE9reW91Zk04bEFlXzE2NjY4ODYxOTQ6MTY2Njg4OTc5NF9WNA" alt="img" style="zoom:80%;" /> |      |
|     DCDC     |      |  1   |      | <img src="https://horizonrobotics.feishu.cn/space/api/box/stream/download/asynccode/?code=YmJmNjBjYzUyMTlhMDc4ZjY2MjdiYmZjZmQ3NmE4MzFfRGlUbWc5a2k0S0hSM0NiRGpKOGhwTlFoUGxlS2MyZkZfVG9rZW46Ym94Y25VTmRoUjNucjJJS2xneWg3TjR6ZXh6XzE2NjY4ODYxOTQ6MTY2Njg4OTc5NF9WNA" alt="img" style="zoom:67%;" /> |      |
|    继电器    |      |  1   |      | <img src="https://horizonrobotics.feishu.cn/space/api/box/stream/download/asynccode/?code=MWZlYzc4MWZhODZmNDZlMDA0OTViM2UxYjZkNmVlNTVfQW40bHE5VDVPemp6QnlLd2VYTDl6ekdRamhxcnFMSXRfVG9rZW46Ym94Y25XeU54S2I0SG83RmYyN3lna0dFdnplXzE2NjY4ODYxOTQ6MTY2Njg4OTc5NF9WNA" alt="img" style="zoom:67%;" /> |      |
|     舵机     |      |  2   |      | <img src="https://horizonrobotics.feishu.cn/space/api/box/stream/download/asynccode/?code=NTQwODg4YjA4M2VkNjk1MGI3YjdhNGY5NjcyNWZkZmZfRGpWNjJDY2FraG1lOEw4MlFQMWRJTnFiMFRZMTVVdFBfVG9rZW46Ym94Y241dENTWXE4OEdBSmNMb2VGTzhpV0FlXzE2NjY4ODYxOTQ6MTY2Njg4OTc5NF9WNA" alt="img" style="zoom:67%;" /> |      |
|   桌面清洁   |      |  1   |      | <img src="docs/pic/-16668862021513.png" alt="img" style="zoom:50%;" /> |      |
