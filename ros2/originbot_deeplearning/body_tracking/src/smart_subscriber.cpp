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

#include "include/smart_subscriber.h"

void SmartMsgSubscriber::topic_callback(
    const ai_msgs::msg::PerceptionTargets::SharedPtr msg) {
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
  RCLCPP_INFO(rclcpp::get_logger("SmartMsgSubscriber"), "%s", ss.str().c_str());

  if (smart_cb_) {
    smart_cb_(msg);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("SmartMsgSubscriber"),
                "smart_cb_ was not set");
  }
}
