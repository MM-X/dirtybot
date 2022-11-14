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

#include <sensor_msgs/msg/image.hpp>
#include <string>

#include "ai_msgs/msg/capture_targets.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "functional"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#ifndef SMART_SUBSCRIBER_H_
#define SMART_SUBSCRIBER_H_

using SmartCbType = std::function<void(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg)>;

class SmartMsgSubscriber : public rclcpp::Node {
 public:
  SmartMsgSubscriber(const std::string &node_name,
                     SmartCbType smart_cb = nullptr)
      : Node(node_name), smart_cb_(smart_cb) {
    this->declare_parameter<std::string>("ai_msg_sub_topic_name",
                                         ai_msg_sub_topic_name_);
    this->get_parameter<std::string>("ai_msg_sub_topic_name",
                                     ai_msg_sub_topic_name_);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("SmartMsgSubscriber"),
                       "Parameter:"
                           << "\n ai_msg_sub_topic_name: "
                           << ai_msg_sub_topic_name_);
    subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        ai_msg_sub_topic_name_,
        10,
        std::bind(
            &SmartMsgSubscriber::topic_callback, this, std::placeholders::_1));
  }

 private:
  void topic_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);

  std::string ai_msg_sub_topic_name_ = "/hobot_hand_gesture_detection";
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr
      subscription_;

  SmartCbType smart_cb_ = nullptr;
};

#endif
