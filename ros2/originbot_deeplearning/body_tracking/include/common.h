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

#ifndef COMMON_H_
#define COMMON_H_

#include <chrono>
#include <fstream>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "ai_msgs/msg/capture_targets.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "include/smart_subscriber.h"
#include "include/time_helper.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;  // NOLINT
using geometry_msgs::msg::Twist;

const float PI = 3.14159;

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

struct Point {
  Point(int x_, int y_) : x(x_), y(y_) {}
  int x;
  int y;
};

struct TrackInfo {
  TrackingStatus tracking_sta = TrackingStatus::INITING;
  uint64_t track_id = 0;
  // ms
  uint64_t frame_ts = 0;
  uint64_t frame_ts_sec = 0;
  uint64_t frame_ts_nanosec = 0;

  int serial_lost_num = 0;
  // x1, y1, x2, y2
  std::vector<int> last_rect{};
  std::vector<int> present_rect{};
  std::vector<Point> present_body_kps{};
  // 是否运动过程中
  std::atomic_bool is_movectrl_running;
  // solution中去掉了face head body的merge功能，原因是merge策略是以head为main
  // id，所以当body未消失，head id发生变化时会导致body id变化 因此需要给body
  // match face/head
  bool has_face_head = false;
  // 连续消失face和head的帧数，即有body无face和head的帧数
  // 只有当robot距离track很近的情况下才会出现face/head消失
  // robot在躲避障碍物过程中，body会和face/head一起消失，因此可以通过调整阈值避免在避障过程中cancel
  // action请求
  uint64_t serial_lost_face_head_num = 0;
  int gesture = 0;

  // track body检测框中心点和robot的y轴负方向的欧拉夹角
  // 机器人的X、Y轴正方向和原点o的关系位置说明：
  /*
      x
      ^
    1 | 2
  y<--o---
    3 | 4
  */
  int angel_with_robot_ = 0;
  // track body检测框中心点和robot的y轴负方向的弧度夹角
  float robot_y_negtive_radian_with_track_ = 0;
  // track和robot的x轴正方向之间的弧度夹角，track在robot逆时针方向值为正，顺时针方向值为负
  // 即robot转向track需要旋转的弧度
  float robot_x_positive_radian_with_track_ = 0;

  // robot前进的距离
  float move_distance_ = 0;
  // 前进的方向: 0 front, 1 back
  int move_direction = 0;
  float move_step = 0.1;
};

struct TrackCfg {
  // true: body, false: hand
  int track_body = 1;
  int activate_wakeup_gesture = 1;
  int wakeup_tracking_gesture_ =
      static_cast<int>(GestureCtrlType::Okay);  // OK手势
  int cancel_tracking_gesture_ =
      static_cast<int>(GestureCtrlType::Palm);  // 手掌手势
  int track_serial_lost_num_thr = 300;
  // unit of measurement is degree
  int activate_robot_rotate_thr = 30;
  // unit of measurement is degree
  int track_lost_protection_angel_thr = 80;
  // unit of measurement is degree
  int track_overmoving_protection_angel_thr = 10;
  // unit of measurement is pixel
  int stop_robot_move_to_top_thr = 20;
  // unit of measurement is pixel
  int activate_robot_move_thr = 5;
  // unit of measurement is pixel
  int activate_robot_move_back_thr = 200;

  float move_step = 0.1;                      // 0.5m
  float activate_robot_move_meter_thr = 1.5;  // meter
  float rotate_step = 0.174;                  // 10 degree

  // 如果body rect宽度超过画面宽度的ratio，认为距离很近，不激活move
  float stop_move_rect_width_ratio_thr = 0.9;
  // 如果body rect宽度小于画面宽度的ratio，认为距离很远，激活move
  // start_move_rect_width_ratio_thr需要小于stop_move_rect_width_ratio_thr
  float start_move_rect_width_ratio_thr = 0.8;

  // 如果小车和正前方距离小于阈值，停止move，单位米
  float collision_avoid_dist_thr = 0.5;
  // check小车正前方和左右各1个扫描点距离的最小值
  int collision_avoid_front_scan_thr = 1;

  // 检测框对应的图片分辨率
  int img_width = 960;
  int img_height = 544;
};

struct LaserScanMsg {
  uint64_t ts = 0;
  sensor_msgs::msg::LaserScan laserscan_data;
};

struct HwGpioCfg {
  uint8_t pin_r = 40;
  uint8_t pin_g = 38;
  uint8_t pin_b = 36;
  uint8_t pin_gnd = 34;
  std::string script_file_path = "install/lib/body_tracking/scripts/gpio.py";
  std::mutex mtx;
  std::string last_set{""};
};
#endif
