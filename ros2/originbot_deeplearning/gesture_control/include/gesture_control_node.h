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

#ifndef GESTURE_STRATEGY_NODE_H
#define GESTURE_STRATEGY_NODE_H

#include <string>
#include "include/common.h"

using geometry_msgs::msg::Twist;
using SmartCbType = std::function<void(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg)>;

class GestureControlNode : public rclcpp::Node {
 public:
  GestureControlNode(const std::string &node_name, SmartCbType smart_cb);

  void RobotCtl(const Twist &msg) const;

 private:
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr
      ai_msg_subscription_ = nullptr;
  void SmartTopicCallback(
      const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);

  std::string ai_msg_sub_topic_name_ = "/hobot_hand_gesture_detection";
  SmartCbType smart_cb_ = nullptr;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr
      smart_subscription_ = nullptr;

  // topic name for turtle sim is "turtle1/cmd_vel" and for robot is "/cmd_vel"
  std::string twist_pub_topic_name_ = "/cmd_vel";
  rclcpp::Publisher<Twist>::SharedPtr twist_publisher_ = nullptr;
};

#endif  // GESTURE_STRATEGY_NODE_H
