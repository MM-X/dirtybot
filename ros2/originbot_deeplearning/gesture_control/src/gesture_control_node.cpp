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

#include "include/gesture_control_node.h"

#include <fstream>
#include <string>

GestureControlNode::GestureControlNode(const std::string& node_name,
                                       SmartCbType smart_cb)
    : Node(node_name), smart_cb_(smart_cb) {
  this->declare_parameter<std::string>("twist_pub_topic_name",
                                       twist_pub_topic_name_);
  this->declare_parameter<std::string>("ai_msg_sub_topic_name",
                                       ai_msg_sub_topic_name_);

  this->get_parameter<std::string>("twist_pub_topic_name",
                                   twist_pub_topic_name_);
  this->get_parameter<std::string>("ai_msg_sub_topic_name",
                                   ai_msg_sub_topic_name_);

  std::stringstream ss;
  ss << "Parameter:"
     << "\n ai_msg_sub_topic_name: " << ai_msg_sub_topic_name_
     << "\n twist_pub_topic_name: " << twist_pub_topic_name_;
  RCLCPP_WARN(rclcpp::get_logger("GestureControlNode"), "%s", ss.str().c_str());

  smart_subscription_ =
      this->create_subscription<ai_msgs::msg::PerceptionTargets>(
          ai_msg_sub_topic_name_,
          10,
          std::bind(&GestureControlNode::SmartTopicCallback,
                    this,
                    std::placeholders::_1));
  twist_publisher_ = this->create_publisher<Twist>(twist_pub_topic_name_, 10);
}

void GestureControlNode::RobotCtl(const Twist& msg) const {
  // std::stringstream ss;
  // ss << "RobotCtl " << msg.angular.x
  // << " " << msg.angular.y
  // << " " << msg.angular.z;
  // static std::ofstream ofs("dump.log");
  // ofs << ss.str() << std::endl;

  twist_publisher_->publish(msg);
}

void GestureControlNode::SmartTopicCallback(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
  std::stringstream ss;
  ss << "Recved ai msg"
     << ", frame_id: " << msg->header.frame_id
     << ", stamp: " << msg->header.stamp.sec << "_" << msg->header.stamp.nanosec
     << ", targets size: " << msg->targets.size() << "\n";
  for (const auto& tar : msg->targets) {
    ss << "\t track_id: " << tar.track_id << ", type: " << tar.type;
    ss << " has roi num: " << tar.rois.size();
    for (const auto& roi : tar.rois) {
      ss << ", roi type: " << roi.type;
    }
    ss << ", has attr num: " << tar.attributes.size();
    for (const auto& attr : tar.attributes) {
      ss << ", attr type: " << attr.type << ", val: " << attr.value;
    }
    ss << "\n";
  }
  RCLCPP_INFO(rclcpp::get_logger("GestureControlNode"), "%s", ss.str().c_str());

  if (smart_cb_) {
    smart_cb_(msg);
  }
}
