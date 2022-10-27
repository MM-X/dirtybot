#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2022, www.guyuehome.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
@作者: 古月居(www.guyuehome.com)
@说明: 发布速度控制话题，控制机器人圆周运动
"""

import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from geometry_msgs.msg import Twist              # 速度话题的消息

"""
创建一个发布者节点
"""
class PublisherNode(Node):
    
    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.pub   = self.create_publisher(Twist, 'cmd_vel', 10)  # 创建发布者对象（消息类型、话题名、队列长度）
        self.timer = self.create_timer(0.5, self.timer_callback)  # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        
    def timer_callback(self):                                     # 创建定时器周期执行的回调函数
        twist = Twist()                                           # 创建一个Twist类型的消息对象
        twist.linear.x  = 0.2                                     # 填充消息对象中的线速度
        twist.angular.z = 0.8                                     # 填充消息对象中的角速度
        self.pub.publish(twist)                                   # 发布话题消息
        self.get_logger().info('Publishing: "linear: %0.2f, angular: %0.2f"' % (twist.linear.x, twist.angular.z))  
        
def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = PublisherNode("draw_circle")              # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
