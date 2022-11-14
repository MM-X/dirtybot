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

#include <fstream>
#include <memory>
#include <sstream>

#include "include/body_tracking.h"

int main(int argc, char *argv[]) {
  std::stringstream ss;
  ss << "\n\tThis is body tracking package.\n\n"
     << "\tgesture strategy usage\n"
     << "\nWake up gesture is \"Okay\".\n"
     << "Cancel gesture is \"Palm\".\n"
     << "Control will be reset if body is lost.\n"
     << "============================================\n";
  std::cout << ss.str() << std::endl;

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto nodes = TrackingManager::Instance()->GetNodes();
  for (auto &node : nodes) {
    exec.add_node(node);
  }

  auto smart_msg_subscriber = std::make_shared<SmartMsgSubscriber>(
      "ai_msg_sub_node",
      std::bind(&TrackingManager::FeedSmart,
                TrackingManager::Instance(),
                std::placeholders::_1));

  exec.add_node(smart_msg_subscriber);
  exec.spin();

  // release node before shutdown!
  TrackingManager::Instance()->Release();

  rclcpp::shutdown();

  std::cout << "tracking node exit!\n";
  return 0;
}
