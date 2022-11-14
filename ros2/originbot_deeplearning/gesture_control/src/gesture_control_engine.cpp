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

#include "include/gesture_control_engine.h"

#include <fstream>
#include <memory>
#include <sstream>
#include <vector>
#include <utility>

std::shared_ptr<GestureControlEngine> GestureControlEngine::Instance() {
  static std::shared_ptr<GestureControlEngine> inst =
      std::shared_ptr<GestureControlEngine>(new GestureControlEngine());
  return inst;
}

GestureControlEngine::GestureControlEngine() {
  RCLCPP_INFO(rclcpp::get_logger("GestureControlEngine"),
              "GestureControlEngine construct");
  start_ = true;

  param_node_ = std::make_shared<ParametersClass>(&track_cfg_);

  gesture_control_node_ = std::make_shared<GestureControlNode>(
      "gesture_control",
      std::bind(&GestureControlEngine::FeedSmart, this, std::placeholders::_1));

  if (!smart_process_task_) {
    smart_process_task_ = std::make_shared<std::thread>([this]() {
      while (rclcpp::ok()) {
        std::unique_lock<std::mutex> lg(smart_queue_mtx_);
        smart_queue_condition_.wait_for(lg, std::chrono::seconds(1), [&]() {
          return !smart_queue_.empty();
        });
        if (smart_queue_.empty() || !rclcpp::ok()) {
          continue;
        }
        auto smart_frame = std::move(smart_queue_.front());
        smart_queue_.pop();
        lg.unlock();
        RunStrategy(smart_frame);
      }

      // 退出前发布停止运动指令，避免程序退出后机器人还一直处于运动状态（如果最后一次收到的指令是启动运动并且运动控制模块没有做超时管理）
      RCLCPP_WARN(rclcpp::get_logger("GestureControlEngine"),
                  "pkg exit! cancel move");
      CancelMove();
    });
  }
}

GestureControlEngine::~GestureControlEngine() {
  RCLCPP_INFO(rclcpp::get_logger("GestureControlEngine"),
              "GestureControlEngine deconstruct");
  start_ = false;

  if (smart_process_task_ && smart_process_task_->joinable()) {
    smart_process_task_->join();
    smart_process_task_ = nullptr;
  }
}

void GestureControlEngine::DoRotate(int direction, float step) {
  last_ctrl_is_cancel_ = false;
  RCLCPP_WARN(rclcpp::get_logger("GestureControlEngine"),
              "do rotate, direction: %d, step: %f",
              direction,
              step);

  int direct = 1;
  if (static_cast<int>(GestureDirectionType::RIGHT) == direction) {
    direct = -1;
  }

  auto twist = std::make_shared<Twist>();
  twist->angular.z = direct * step * 4;
  FeedMovePoseMsg(twist);

  RCLCPP_INFO(rclcpp::get_logger("GestureControlEngine"),
              "present frame_ts_ms: %llu",
              track_info_.frame_ts_ms);
}

void GestureControlEngine::DoMove(int direction, float step) {
  last_ctrl_is_cancel_ = false;
  RCLCPP_WARN(rclcpp::get_logger("GestureControlEngine"),
              "do move, direction: %d, step: %f",
              direction,
              step);
  auto twist = std::make_shared<Twist>();
  if (0 == direction) {
    twist->linear.x += step * 2;
  } else if (1 == direction) {
    twist->linear.x -= step * 2;
  } else if (2 == direction) {
    twist->linear.y += step;
  } else if (3 == direction) {
    twist->linear.y -= step;
  }

  FeedMovePoseMsg(twist);
}

void GestureControlEngine::CancelMove() {
  if (last_ctrl_is_cancel_) return;
  last_ctrl_is_cancel_ = true;
  RCLCPP_WARN(rclcpp::get_logger("GestureControlEngine"), "cancel move");
  auto twist = std::make_shared<Twist>();
  twist->linear.x = 0;
  twist->linear.y = 0;
  twist->linear.z = 0;
  twist->angular.x = 0;
  twist->angular.y = 0;
  twist->angular.z = 0;
  FeedMovePoseMsg(twist);
}

void GestureControlEngine::FeedMovePoseMsg(const Twist::SharedPtr &pose) {
  if (gesture_control_node_ && pose) {
    gesture_control_node_->RobotCtl(*pose);
  }
}

void GestureControlEngine::ProcessSmart(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &ai_msg) {
  // update track info
  // if the track is lost, reset track info

  if (TrackingStatus::TRACKING == track_info_.tracking_sta) {
    RCLCPP_DEBUG(rclcpp::get_logger("GestureControlEngine"), "tracking");

    // update status
    // find track
    bool find_hand = false;
    std::vector<int> present_rect;
    present_rect.resize(4);
    int gesture = 0;

    for (const auto target : ai_msg->targets) {
      RCLCPP_DEBUG(
          rclcpp::get_logger("GestureControlEngine"),
          "target.rois.size: %d, target.track_id: %d, track_info_.track_id: %d",
          target.rois.size(),
          target.track_id,
          track_info_.track_id);
      if (target.track_id != track_info_.track_id) continue;
      // target的track_id和和控制手的id一致
      // 更新检测框
      for (const auto &roi : target.rois) {
        RCLCPP_DEBUG(rclcpp::get_logger("GestureControlEngine"),
                     "roi.type: %s",
                     roi.type.c_str());
        if ("hand" != roi.type) {
          continue;
        }

        RCLCPP_DEBUG(rclcpp::get_logger("GestureControlEngine"),
                     "hand roi x_offset: %d y_offset: %d width: %d height: %d",
                     roi.rect.x_offset,
                     roi.rect.y_offset,
                     roi.rect.width,
                     roi.rect.height);

        present_rect[0] = roi.rect.x_offset;
        present_rect[1] = roi.rect.y_offset;
        present_rect[2] = roi.rect.x_offset + roi.rect.width;
        present_rect[3] = roi.rect.y_offset + roi.rect.height;

        find_hand = true;
        break;
      }

      if (!find_hand) {
        // 此track没有hand
        continue;
      }

      // 更新手势
      for (const auto &attr : target.attributes) {
        RCLCPP_INFO(rclcpp::get_logger("GestureControlEngine"),
                    "track_id: %d, attr type: %s, val: %d",
                    target.track_id,
                    attr.type.c_str(),
                    attr.value);
        if ("gesture" == attr.type) {
          gesture = attr.value;
        }
      }

      // 查找结束
      break;
    }

    // 更新当前帧的信息
    track_info_.frame_ts_ms = ai_msg->header.stamp.sec * 1000 +
                              ai_msg->header.stamp.nanosec / 1000 / 1000;
    track_info_.gesture = gesture;
    if (find_hand) {
      if (1 == track_cfg_.activate_wakeup_gesture) {
        if (gesture == static_cast<int>(track_cfg_.reset_gesture)) {
          RCLCPP_WARN(rclcpp::get_logger("GestureControlEngine"),
                      "frame_ts_ms %llu, track id: %d recved reset gesture: %d",
                      track_info_.frame_ts_ms,
                      track_info_.track_id,
                      gesture);
          track_info_.tracking_sta = TrackingStatus::INITING;
          return;
        }
      }

      RCLCPP_INFO(rclcpp::get_logger("GestureControlEngine"),
                  "update track id: %d",
                  track_info_.track_id);
      // update
      track_info_.serial_lost_num = 0;
      track_info_.present_rect = present_rect;
    } else {
      track_info_.serial_lost_num++;
      if (static_cast<int>(track_info_.serial_lost_num) >=
          track_cfg_.track_serial_lost_num_thr) {
        // track lost
        track_info_.tracking_sta = TrackingStatus::LOST;
        RCLCPP_WARN(rclcpp::get_logger("GestureControlEngine"),
                    "track id: %d serial lost num exceed limit: %d",
                    track_info_.track_id,
                    track_cfg_.track_serial_lost_num_thr);
      }
    }
  } else if (TrackingStatus::INITING == track_info_.tracking_sta ||
             TrackingStatus::LOST == track_info_.tracking_sta) {
    // find the max hand accoding rect pixel from all hands, which have gesture
    if (ai_msg->targets.empty()) {
      return;
    }

    bool has_hand = false;
    uint64_t track_id;
    int gesture = 0;
    int found_gesture = -1;
    std::vector<int> present_rect;
    std::vector<int> found_rect;

    for (const auto &target : ai_msg->targets) {
      RCLCPP_DEBUG(rclcpp::get_logger("GestureControlEngine"),
                   "target.rois.size: %d",
                   target.rois.size());

      // 更新检测框
      for (const auto &roi : target.rois) {
        RCLCPP_DEBUG(rclcpp::get_logger("GestureControlEngine"),
                     "roi.type: %s",
                     roi.type.c_str());
        if ("hand" != roi.type) {
          continue;
        }

        RCLCPP_DEBUG(rclcpp::get_logger("GestureControlEngine"),
                     "recv track_id: %d, hand roi x_offset: %d y_offset: %d "
                     "width: %d height: %d",
                     target.track_id,
                     roi.rect.x_offset,
                     roi.rect.y_offset,
                     roi.rect.width,
                     roi.rect.height);
        found_rect.push_back(roi.rect.x_offset);
        found_rect.push_back(roi.rect.y_offset);
        found_rect.push_back(roi.rect.x_offset + roi.rect.width);
        found_rect.push_back(roi.rect.y_offset + roi.rect.height);
        break;
      }

      // 更新手势
      for (const auto &attr : target.attributes) {
        RCLCPP_INFO(rclcpp::get_logger("GestureControlEngine"),
                    "track_id: %d, attr type: %s, val: %d",
                    target.track_id,
                    attr.type.c_str(),
                    attr.value);
        if ("gesture" == attr.type) {
          found_gesture = attr.value;
          break;
        }
      }

      if (found_gesture < 0 || found_rect.empty()) {
        continue;
      }

      if (1 == track_cfg_.activate_wakeup_gesture) {
        if (found_gesture != static_cast<int>(track_cfg_.wakeup_gesture)) {
          continue;
        }
      } else {
        if (static_cast<int>(GestureCtrlType::Awesome) != found_gesture &&
            static_cast<int>(GestureCtrlType::Victory) != found_gesture &&
            static_cast<int>(GestureCtrlType::ThumbRight) != found_gesture &&
            static_cast<int>(GestureCtrlType::ThumbLeft) != found_gesture) {
          continue;
        }
      }

      if (!has_hand) {
        RCLCPP_INFO(rclcpp::get_logger("GestureControlEngine"),
                    "first find hand");
        has_hand = true;
        track_id = target.track_id;
        gesture = found_gesture;
        present_rect = found_rect;
      } else if ((present_rect[2] - present_rect[0]) <
                 (found_rect[2] - found_rect[1])) {
        // 如果当前帧中有多个hand有手势，选择最大宽度的hand
        RCLCPP_INFO(rclcpp::get_logger("GestureControlEngine"),
                    "update found hand");
        track_id = target.track_id;
        gesture = found_gesture;
        present_rect = found_rect;
      }
    }

    // the found hand will control robot using gesture
    if (has_hand) {
      track_info_.tracking_sta = TrackingStatus::TRACKING;
      track_info_.track_id = track_id;
      track_info_.gesture = gesture;
      track_info_.present_rect = present_rect;
      track_info_.frame_ts_ms = ai_msg->header.stamp.sec * 1000 +
                                ai_msg->header.stamp.nanosec / 1000 / 1000;

      std::stringstream ss;
      ss << "Gesture contrl start!"
         << ", track_id: " << track_info_.track_id
         << ", frame_ts_ms: " << track_info_.frame_ts_ms
         << ", tracking_sta(0:INITING, 1:TRACKING, 2:LOST): "
         << static_cast<int>(track_info_.tracking_sta)
         << ", gesture: " << track_info_.gesture;
      RCLCPP_WARN(
          rclcpp::get_logger("GestureControlEngine"), "%s", ss.str().data());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("GestureControlEngine"),
              "tracking_sta: %d (0:INITING, 1:TRACKING, 2:LOST), track_id: %d, "
              "gesture: %d, frame_ts_ms: %llu",
              track_info_.tracking_sta,
              track_info_.track_id,
              track_info_.gesture,
              track_info_.frame_ts_ms);
}

void GestureControlEngine::RunStrategy(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg) {
  if (!msg || !rclcpp::ok()) {
    return;
  }

  ProcessSmart(msg);

  if (TrackingStatus::TRACKING != track_info_.tracking_sta) {
    RCLCPP_DEBUG(rclcpp::get_logger("GestureControlEngine"), "track is lost");
    CancelMove();
    return;
  }

  if (static_cast<int>(GestureCtrlType::Awesome) != track_info_.gesture &&
      static_cast<int>(GestureCtrlType::Victory) != track_info_.gesture &&
      static_cast<int>(GestureCtrlType::ThumbRight) != track_info_.gesture &&
      static_cast<int>(GestureCtrlType::ThumbLeft) != track_info_.gesture) {
    CancelMove();
    return;
  }

  RCLCPP_WARN(rclcpp::get_logger("GestureControlEngine"),
              "frame_ts_ms: %llu, track_id: %d, tracking_sta: %d, gesture: %d",
              track_info_.frame_ts_ms,
              track_info_.track_id,
              static_cast<int>(track_info_.tracking_sta),
              static_cast<int>(track_info_.gesture));

  if (static_cast<int>(GestureCtrlType::Awesome) == track_info_.gesture) {
    // move front
    DoMove(static_cast<int>(GestureDirectionType::FRONT), track_cfg_.move_step);
    return;
  } else if (static_cast<int>(GestureCtrlType::Victory) ==
             track_info_.gesture) {
    // move back
    DoMove(static_cast<int>(GestureDirectionType::BACK), track_cfg_.move_step);
    return;
  } else if (static_cast<int>(GestureCtrlType::ThumbRight) ==
             track_info_.gesture) {
    // rotate right
    DoRotate(static_cast<int>(GestureDirectionType::RIGHT),
             track_cfg_.rotate_step);
    return;
  } else if (static_cast<int>(GestureCtrlType::ThumbLeft) ==
             track_info_.gesture) {
    // rotate left
    DoRotate(static_cast<int>(GestureDirectionType::LEFT),
             track_cfg_.rotate_step);
    return;
  }

  CancelMove();
  return;
}

void GestureControlEngine::FeedSmart(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg) {
  std::unique_lock<std::mutex> lg(smart_queue_mtx_);
  smart_queue_.push(msg);
  if (smart_queue_.size() > queue_len_limit_) {
    RCLCPP_ERROR(rclcpp::get_logger("GestureControlEngine"),
                 "smart queue len exceed limit: %d",
                 queue_len_limit_);
    smart_queue_.pop();
  }
  smart_queue_condition_.notify_one();
  lg.unlock();
}
