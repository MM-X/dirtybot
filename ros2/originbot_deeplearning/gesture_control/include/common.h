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

#ifndef COMMON_H
#define COMMON_H

#include <chrono>
#include <memory>
#include <queue>
#include <vector>
#include <string>

#include "ai_msgs/msg/capture_targets.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "functional"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;  // NOLINT
using geometry_msgs::msg::Twist;

enum class GestureDirectionType { FRONT = 0, BACK, LEFT, RIGHT };

enum class GestureCtrlType {
  ThumbUp = 2,  // 点赞
  Victory = 3,
  Palm = 5,         // 手掌
  Okay = 11,        //  OK手势
  ThumbRight = 12,  // 大拇指向左
  ThumbLeft = 13,   // 大拇指向右
  Awesome = 14      // 666手势
};

enum class TrackingStatus { INITING = 0, TRACKING, LOST };

struct TrackInfo {
  TrackingStatus tracking_sta = TrackingStatus::INITING;
  uint64_t track_id = 0;
  uint64_t serial_lost_num = 0;
  uint64_t frame_ts_ms = 0;
  std::vector<int> present_rect{};
  int gesture = 0;
  int gesture_direction = -1;
};

struct TrackCfg {
  const GestureCtrlType wakeup_gesture = GestureCtrlType::Okay;  // OK手势
  const GestureCtrlType reset_gesture = GestureCtrlType::Palm;   // 手掌手势
  int activate_wakeup_gesture = 0;
  int track_serial_lost_num_thr = 100;
  float move_step = 0.1;
  float rotate_step = 0.5;
};

#endif  // COMMON_H
