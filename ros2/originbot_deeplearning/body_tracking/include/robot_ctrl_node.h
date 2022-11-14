// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROBOT_CTRL_NODE_H_
#define ROBOT_CTRL_NODE_H_
#include <string>

#include "include/common.h"

class RobotCmdVelNode : public rclcpp::Node {
 public:
  explicit RobotCmdVelNode(const std::string &node_name) : Node(node_name) {
    RCLCPP_INFO(rclcpp::get_logger("RobotCmdVelNode"),
                "RobotCmdVelNode construct");
    twist_publisher_ = this->create_publisher<Twist>(twist_pub_topic_name_, 10);
  }

  void RobotCtl(const Twist &msg) const {
    std::stringstream ss;
    ss << "RobotCtl"
       << ", angular: " << msg.angular.x << " " << msg.angular.y << " "
       << msg.angular.z << ", linear: " << msg.linear.x << " " << msg.linear.y
       << " " << msg.linear.z
       << ", pub twist ts: " << TimeHelper::GetCurrentTimestampMicroSec();
    RCLCPP_WARN(rclcpp::get_logger("RobotCmdVelNode"), "%s", ss.str().data());

    twist_publisher_->publish(msg);
  }

 private:
  rclcpp::Publisher<Twist>::SharedPtr twist_publisher_ = nullptr;
  // topic name for turtle sim is "turtle1/cmd_vel" and for robot is "/cmd_vel"
  const std::string twist_pub_topic_name_ = "/cmd_vel";
};

#endif
