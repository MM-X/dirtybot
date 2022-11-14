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

#ifndef GESTURE_CONTROL_ENGINE_H
#define GESTURE_CONTROL_ENGINE_H

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "include/common.h"
#include "include/gesture_control_node.h"
#include "include/param_node.h"

class GestureControlEngine {
 public:
  static std::shared_ptr<GestureControlEngine> Instance();
  ~GestureControlEngine();

  void FeedSmart(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg);
  void UpdateRobotPose(const geometry_msgs::msg::Twist::SharedPtr &pose);

  std::vector<std::shared_ptr<rclcpp::Node>> GetNodes() {
    std::vector<std::shared_ptr<rclcpp::Node>> node_ptrs;
    node_ptrs.push_back(param_node_);
    node_ptrs.push_back(gesture_control_node_);
    return node_ptrs;
  }

 private:
  GestureControlEngine();

  void RunStrategy(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg);
  void ProcessSmart(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg);
  // rotate robot
  // direction: 0 left, 1 right; unit of step is radian
  void DoRotate(int direction, float step = 0.5);
  // direction: 0 front, 1 back, 2 left, 3 right; unit of step is m/s
  void DoMove(int direction, float step = 0.05);
  void CancelMove();
  // push dest world position msg to queue, which will be pub using action
  void FeedMovePoseMsg(const Twist::SharedPtr &pose);

  std::shared_ptr<ParametersClass> param_node_ = nullptr;
  std::shared_ptr<GestureControlNode> gesture_control_node_ = nullptr;

  TrackInfo track_info_;
  bool start_ = false;

  size_t queue_len_limit_ = 10;
  std::queue<ai_msgs::msg::PerceptionTargets::ConstSharedPtr> smart_queue_;
  std::mutex smart_queue_mtx_;
  std::condition_variable smart_queue_condition_;

  std::shared_ptr<std::thread> smart_process_task_ = nullptr;
  std::shared_ptr<std::thread> feed_pose_task_ = nullptr;

  TrackCfg track_cfg_;
  bool last_ctrl_is_cancel_ = false;
};

#endif  // GESTURE_CONTROL_ENGINE_H
