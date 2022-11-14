# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(body_tracking)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_compile_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# for msg generation
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) 
find_package(ai_msgs REQUIRED) 

ament_export_dependencies(rosidl_default_runtime)

include_directories(
    ${PROJECT_SOURCE_DIR}/
    ${PROJECT_SOURCE_DIR}/include
    )

file(GLOB_RECURSE HEADERFILES
  ${PROJECT_SOURCE_DIR}/include/*.h
  )

link_directories(
  )

add_executable(${PROJECT_NAME}
  src/smart_subscriber.cpp
  src/main.cpp
  src/body_tracking.cpp
  src/util.cpp
)

target_link_libraries(${PROJECT_NAME}
  opencv_world
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_action"
  "ai_msgs")

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/scripts/
  DESTINATION lib/${PROJECT_NAME}/scripts)

install(FILES
  ${PROJECT_SOURCE_DIR}/README.md
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/images/
  DESTINATION lib/${PROJECT_NAME}/images)

ament_package()
