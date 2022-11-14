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

#ifndef TRACKING_STRATEGY_H_
#define TRACKING_STRATEGY_H_

#include <chrono>
#include <fstream>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "ai_msgs/msg/capture_targets.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "include/common.h"
#include "include/param_node.h"
#include "include/robot_ctrl_node.h"
#include "include/smart_subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class TrackingManager {
 public:
  static std::shared_ptr<TrackingManager> Instance();
  ~TrackingManager();

  void Release();
  void FeedSmart(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg);
  std::vector<std::shared_ptr<rclcpp::Node>> GetNodes();
  const TrackCfg &GetTrackCfg() const;

 private:
  TrackingManager();

  void RunTrackingStrategy(
      const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg);
  void TrackingWithoutNavStrategy(
      const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg);
  // avoid track lost from camera on robot
  void RunTrackLostProtectionStrategy();
  // avoid robot move far away
  void RunOverMovingProtectionStrategy();
  void ProcessSmart(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg);
  // 只更新present_rect和frame_id
  void UpdateSmart(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg);
  // check if last command send to robot is complete
  bool RobotCommandIsRunning();
  // cal the angle of robot and track
  void UpdateTrackAngle();
  // check if robot needs rotate
  bool RotateSwitch();

  void CancelMove();

  void DoRotateMove();

  // check if robot needs move
  // 只使用视觉检测结果数据，判断是否需要激活跟随
  bool TrackingSwitchWithVision();

  // 设置gpio led灯，1点亮，0关闭
  void SetLed(int r, int g, int b);

  std::mutex robot_strategy_mtx_;
  // 如果上一帧智能数据的策略未处理完，当前智能数据不处理
  std::atomic_bool last_frame_done_;

  std::shared_ptr<ParametersClass> param_node_ = nullptr;
  std::shared_ptr<RobotCmdVelNode> robot_cmdvel_node_ = nullptr;

  TrackInfo track_info_;
  TrackCfg track_cfg_;
  HwGpioCfg hw_gpio_cfg_;

  bool start_ = false;

  size_t queue_len_limit_ = 2;
  std::queue<ai_msgs::msg::PerceptionTargets::ConstSharedPtr> smart_queue_;
  std::mutex smart_queue_mtx_;
  std::condition_variable smart_queue_condition_;

  std::shared_ptr<std::thread> smart_process_task_ = nullptr;

  bool last_cmdvel_is_cancel_ = false;
  // 0: rotate, 1: move, 2: rotate&move
  int last_cmdvel_type_ = -1;
};
#endif
