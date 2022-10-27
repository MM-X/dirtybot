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
@说明: 发送控制LED的请求, 完成对LED的控制
"""

import time
import rclpy                                                                      # ROS2 Python接口库
from rclpy.node   import Node                                                     # ROS2 节点类
from originbot_msgs.srv import OriginbotLed                                       # 自定义的服务接口

class serverClient(Node):
    def __init__(self, name):
        super().__init__(name)                                                    # ROS2节点父类初始化
        self.client = self.create_client(OriginbotLed, 'originbot_led')           # 创建服务客户端对象（服务接口类型，服务名）
        while not self.client.wait_for_service(timeout_sec=1.0):                  # 循环等待服务器端成功启动
            self.get_logger().info('service not available, waiting again...') 
        self.request = OriginbotLed.Request()                                     # 创建服务请求的数据对象
                    
    def send_request(self, led_on):                                               # 创建一个发送服务请求的函数
        self.request.on = led_on
        self.future = self.client.call_async(self.request)                        # 异步方式发送服务请求

def main(args=None):
    rclpy.init(args=args)                                                         # ROS2 Python接口初始化
    node = serverClient("control_led")                                            # 创建ROS2节点对象并进行初始化
    
    led_on= True
    while rclpy.ok():                                                             # ROS2系统正常运行
        node.send_request(led_on)                                                 # 发送服务请求
        rclpy.spin_once(node)                                                     # 循环执行一次节点
        
        led_on = not led_on
        time.sleep(3)

    node.destroy_node()                                                           # 销毁节点对象
    rclpy.shutdown()                                                              # 关闭ROS2 Python接口
