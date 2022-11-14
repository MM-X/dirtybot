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

#include <cstdio>

#include "include/gesture_control_engine.h"

int main(int argc, char** argv) {
  std::stringstream ss;
  ss << "\n\tThis is gesture control package.\n\n"
     << "============================================\n"
     << "\tgesture control usage\n"
     << "\nWake up gesture is \"Okay\".\n"
     << "Reset gesture is \"Palm\".\n"
     << "Gesture control definitions are:\n"
     << "\t\"Awesome\": move front. (close from controler)\n"
     << "\t\"Victory\": move back. (far from controler)\n"
     << "\t\"ThumbRight\": rotate robot to right. (rotate to the thumb "
        "orientation)\n"
     << "\t\"ThumbLeft\": rotate robot to left. (rotate to the thumb "
        "orientation)\n"
     << "Control will be paused if gesture is lost.\n"
     << "Control will be reset if hand is lost or the gesture is \"Palm\".\n"
     << "============================================\n";
  std::cout << ss.str() << std::endl;

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto nodes = GestureControlEngine::Instance()->GetNodes();
  for (auto& node : nodes) {
    exec.add_node(node);
  }
  exec.spin();
  rclcpp::shutdown();

  return 0;
}
