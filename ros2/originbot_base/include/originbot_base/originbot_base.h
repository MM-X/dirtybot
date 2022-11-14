/***********************************************************************
Copyright (c) 2022, www.guyuehome.com

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include <chrono>
#include <memory>
#include <iostream>
#include <cstring>
#include <string>
#include <cmath>
#include <thread>
#include <algorithm>
#include <csignal>
#include <stdlib.h>
#include <serial/serial.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "originbot_msgs/msg/originbot_status.hpp"
#include "originbot_msgs/srv/originbot_servo.hpp"
#include "originbot_msgs/srv/originbot_voice.hpp"
#include "originbot_msgs/srv/originbot_pid.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

#define ORIGINBOT_WHEEL_TRACK  (0.11)

// originbot protocol data format
typedef struct {
    uint8_t header;
    uint8_t id;
    uint8_t length;
    uint8_t data[6];
    uint8_t check;
    uint8_t tail;
} DataFrame;

typedef struct {
    float acceleration_x;
    float acceleration_y;
    float acceleration_z;
    float angular_x;
    float angular_y;
    float angular_z;
    float roll;
    float pitch;
    float yaw;
} DataImu;

typedef struct {
    float battery_voltage;
    float tof_distance;
    uint16_t voice_cmd;
    bool voice_on;
    bool servo_on;
} RobotStatus;

enum {
    FRAME_ID_MOTION       = 0x01,
    FRAME_ID_VELOCITY     = 0x02,
    FRAME_ID_ACCELERATION = 0x03,
    FRAME_ID_ANGULAR      = 0x04,
    FRAME_ID_EULER        = 0x05,
    FRAME_ID_SENSOR       = 0x06,
    FRAME_ID_HMI          = 0x07,
};

class OriginbotBase : public rclcpp::Node
{
public:
    OriginbotBase(std::string nodeName);
    ~OriginbotBase();

    void driver_loop();
    
private:
    void readRawData();
    bool checkDataFrame(DataFrame &frame);
    void processDataFrame(DataFrame &frame);

    void processVelocityData(DataFrame &frame);
    void processAngularData(DataFrame &frame);
    void processAccelerationData(DataFrame &frame);
    void processEulerData(DataFrame &frame);
    void processSensorData(DataFrame &frame);

    double imu_conversion(uint8_t data_high, uint8_t data_low);
    bool   imu_calibration();
    double degToRad(double deg);

    void odom_publisher(float vx, float vth);
    void imu_publisher();

    bool voice_control(uint16_t req);
    bool shovel_control(uint8_t angle);

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void voice_callback(const std::shared_ptr<originbot_msgs::srv::OriginbotVoice::Request>  request,
                               std::shared_ptr<originbot_msgs::srv::OriginbotVoice::Response> response);
    void shovel_callback(const std::shared_ptr<originbot_msgs::srv::OriginbotServo::Request>  request,
                            std::shared_ptr<originbot_msgs::srv::OriginbotServo::Response> response);
    void pid_callback(const std::shared_ptr<originbot_msgs::srv::OriginbotPID::Request>  request,
                            std::shared_ptr<originbot_msgs::srv::OriginbotPID::Response> response);

    void timer_100ms_callback();

private:
    serial::Serial serial_;
    rclcpp::Time current_time_;
    float odom_x_=0.0, odom_y_=0.0, odom_th_=0.0;

    float correct_factor_vx_ = 1.0;
    float correct_factor_vth_ = 1.0;    
    
    std::shared_ptr<std::thread> read_data_thread_;
    DataImu imu_data_;
    RobotStatus robot_status_;

    rclcpp::TimerBase::SharedPtr timer_100ms_;
    bool auto_stop_on_ = true;
    bool use_imu_ = false;
    unsigned int auto_stop_count_ = 0;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<originbot_msgs::msg::OriginbotStatus>::SharedPtr status_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
   
    rclcpp::Service<originbot_msgs::srv::OriginbotVoice>::SharedPtr voice_service_;
    rclcpp::Service<originbot_msgs::srv::OriginbotServo>::SharedPtr servo_service_;
    rclcpp::Service<originbot_msgs::srv::OriginbotPID>::SharedPtr pid_service_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};