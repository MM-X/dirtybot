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

#ifndef TIMER_HELPER__H_
#define TIMER_HELPER__H_

#include <chrono>

class TimeHelper {
 public:
  static uint64_t GetCurrentTimestampMicroSec() {
    auto time_now = std::chrono::system_clock::now();
    auto duration_in_micro_sec =
        std::chrono::duration_cast<std::chrono::microseconds>(
            time_now.time_since_epoch());
    return static_cast<uint64_t>(duration_in_micro_sec.count());
  }

  static uint64_t GetCurrentTimestampMillSec() {
    auto time_now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        time_now.time_since_epoch());
    return static_cast<uint64_t>(duration.count());
  }
};
#endif  // TIMER_HELPER__H_
