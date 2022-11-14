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

#include "originbot_base/originbot_base.h"

OriginbotBase::OriginbotBase(std::string nodeName) : Node(nodeName)
{
    // 加载参数
    std::string port_name="ttyS3";    
    this->declare_parameter("port_name");           //声明及获取串口号参数
    this->get_parameter_or<std::string>("port_name", port_name, "ttyS3");
    this->declare_parameter("correct_factor_vx");   //声明及获取线速度校正参数
    this->get_parameter_or<float>("correct_factor_vx", correct_factor_vx_, 1.0);
    this->declare_parameter("correct_factor_vth");  //声明及获取角速度校正参数
    this->get_parameter_or<float>("correct_factor_vth", correct_factor_vth_, 1.0);
    this->declare_parameter("auto_stop_on");        //声明及获取自动停车功能的开关值
    this->get_parameter_or<bool>("auto_stop_on", auto_stop_on_, true);
    this->declare_parameter("use_imu");             //声明是否使用imu
    this->get_parameter_or<bool>("use_imu", use_imu_, false);
    
    // 打印加载的参数值
    printf("Loading parameters: \n \
            - port name: %s\n \
            - correct factor vx: %0.4f\n \
            - correct factor vth: %0.4f\n \
            - auto stop on: %d\n \
            - use imu: %d\n",\
            port_name.c_str(), correct_factor_vx_, correct_factor_vth_, auto_stop_on_, use_imu_); 

    // 创建里程计、机器人状态的发布者
    odom_publisher_   = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    status_publisher_ = this->create_publisher<originbot_msgs::msg::OriginbotStatus>("originbot_status", 10);

    // 创建速度指令的订阅者
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&OriginbotBase::cmd_vel_callback, this, _1));
    
    // 创建语音指令和Servo的服务
    voice_service_ = this->create_service<originbot_msgs::srv::OriginbotVoice>("originbot_voice", std::bind(&OriginbotBase::voice_callback, this, _1, _2));
    servo_service_ = this->create_service<originbot_msgs::srv::OriginbotServo>("originbot_servo", std::bind(&OriginbotBase::shovel_callback, this, _1, _2));
    pid_service_ = this->create_service<originbot_msgs::srv::OriginbotPID>("originbot_pid", std::bind(&OriginbotBase::pid_callback, this, _1, _2));

    // 创建TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 控制器与扩展驱动板卡的串口配置与通信
    try
    {
        serial_.setPort("/dev/" + port_name);                            //选择要开启的串口号
        serial_.setBaudrate(115200);                                     //设置波特率
        serial::Timeout timeOut = serial::Timeout::simpleTimeout(2000);  //超时等待
        serial_.setTimeout(timeOut);                                     
        serial_.open();                                                  //开启串口
    }
    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "originbot can not open serial port,Please check the serial port cable! "); //如果开启串口失败，打印错误信息
    }

    // 如果串口打开，则驱动读取数据的线程
    if (serial_.isOpen())
    {
        RCLCPP_INFO(this->get_logger(), "originbot serial port opened"); //串口开启成功提示

        // 启动一个新线程读取并处理串口数据
        read_data_thread_ = std::shared_ptr<std::thread>(
            new std::thread(std::bind(&OriginbotBase::readRawData, this)));
    }

    if(use_imu_)
    {
        // 创建IMU的话题发布者
        imu_publisher_    = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

        // IMU初始化标定
        if(imu_calibration())
        {
            // usleep(200000);    //确保标定完成
            RCLCPP_INFO(this->get_logger(), "IMU calibration ok.");
        }
    }

    // 设置Servo灯的初始状态
    robot_status_.servo_on = true;

    // 启动一个100ms的定时器，处理订阅者之外的其他信息
    timer_100ms_ = this->create_wall_timer(
      100ms, std::bind(&OriginbotBase::timer_100ms_callback, this));

    // 初始化完成，蜂鸣器响1s，并输出日志
    voice_control(true);
    // usleep(500000);
    voice_control(false);

    RCLCPP_INFO(this->get_logger(), "OriginBot Start, enjoy it.");
}

OriginbotBase::~OriginbotBase()
{
    serial_.close();
}

void OriginbotBase::readRawData()
{
    uint8_t rx_data = 0;
    DataFrame frame;

    while (rclcpp::ok()) 
    {
        // 读取一个字节数据，寻找帧头
        auto len = serial_.read(&rx_data, 1);
        if (len < 1)
            continue;
        
        // printf("Frame raw data: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n", 
        //                 frame.header, frame.id, frame.length, frame.data[0], frame.data[1], frame.data[2], 
        //                 frame.data[3], frame.data[4], frame.data[5], frame.check, frame.tail);
        // 发现帧头后开始处理数据帧
        if(rx_data == 0x55)
        {
            // 读取完整的数据帧
            serial_.read(&frame.id, 10);

            // 判断帧尾是否正确
            if(frame.tail != 0xbb)
            {
                RCLCPP_WARN(this->get_logger(), "Data frame tail error!"); 
                printf("Frame raw data[Error]: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n", 
                        frame.header, frame.id, frame.length, frame.data[0], frame.data[1], frame.data[2], 
                        frame.data[3], frame.data[4], frame.data[5], frame.check, frame.tail);
                continue;
            }

            frame.header = 0x55;
            
            // 帧校验
            if(checkDataFrame(frame))
            {
                // printf("Check result = %d", checkDataFrame(frame));
                // printf("Frame raw data: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n", 
                //         frame.header, frame.id, frame.length, frame.data[0], frame.data[1], frame.data[2], 
                //         frame.data[3], frame.data[4], frame.data[5], frame.check, frame.tail);

                //处理帧数据
                processDataFrame(frame);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Data frame check failed!"); 
            }
        }
    }
}

bool OriginbotBase::checkDataFrame(DataFrame &frame)
{    
    if(((frame.data[0] + frame.data[1] + frame.data[2] + 
        frame.data[3] + frame.data[4] + frame.data[5]) & 0xff) == frame.check)
        return true;
    else
        return false;
}

void OriginbotBase::processDataFrame(DataFrame &frame)
{
    // 根据数据帧的ID，对应处理帧数据
    switch(frame.id)
    {
    case FRAME_ID_VELOCITY:
        processVelocityData(frame);
        break;
    case FRAME_ID_ACCELERATION:
        processAccelerationData(frame);
        break;
    case FRAME_ID_ANGULAR:
        processAngularData(frame);
        break;
    case FRAME_ID_EULER:
        processEulerData(frame);
        break;
    case FRAME_ID_SENSOR:
        processSensorData(frame);
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Frame ID Error[%d]", frame.id);
        break;
    }
}

void OriginbotBase::processVelocityData(DataFrame &frame)
{
    //RCLCPP_INFO(this->get_logger(), "Process velocity data");

    float left_speed = 0.0, right_speed = 0.0;
    float vx = 0.0, vth = 0.0;
    float delta_th = 0.0, delta_x = 0.0, delta_y = 0.0;
    static float last_theta = 0.0;

    // 计算两个周期之间的时间差
    static rclcpp::Time last_time_ = this->now();
    current_time_ = this->now();

    float dt = (current_time_.seconds() - last_time_.seconds());
    last_time_ = current_time_;    

    // 获取机器人两侧轮子的速度，完成单位转换mm/s --> m/s
    uint16_t dataTemp = frame.data[2];
    float speedTemp = (float)((dataTemp << 8) | frame.data[1]);
    if (frame.data[0] == 0)
        left_speed = -1.0 * speedTemp / 1000.0;
    else
        left_speed = speedTemp / 1000.0;

    dataTemp = frame.data[5];
    speedTemp = (float)((dataTemp << 8) | frame.data[4]);
    if (frame.data[3] == 0)
        right_speed = -1.0 * speedTemp / 1000.0;
    else
        right_speed = speedTemp / 1000.0;

    // 通过两侧轮子的速度，计算机器人整体的线速度和角速度，通过校正参数做校准
    vx  = correct_factor_vx_  * (left_speed  + right_speed) / 2;                    // m/s
    vth = correct_factor_vth_ * (right_speed - left_speed) / ORIGINBOT_WHEEL_TRACK; // rad/s

    //RCLCPP_INFO(this->get_logger(), "dt=%f left_speed=%f right_speed=%f vx=%f vth=%f", dt, left_speed, right_speed, vx, vth);

    // 计算里程计单周期内的姿态
    if (use_imu_) {
        delta_th = imu_data_.yaw - last_theta;
    } else {
        delta_th = vth * dt;
    }
    last_theta = imu_data_.yaw;
    delta_x = vx * cos(odom_th_ + delta_th / 2.0f) * dt;
    delta_y = vx * sin(odom_th_ + delta_th / 2.0f) * dt;
    
    // 计算里程计的累积姿态
    odom_x_  += delta_x;
    odom_y_  += delta_y;
    odom_th_ += delta_th;
    
    // 校正姿态角度，让机器人处于-180~180度之间
    if(odom_th_ > M_PI) 
        odom_th_ -= M_PI*2;
    else if(odom_th_ < (-M_PI)) 
        odom_th_ += M_PI*2;

    // RCLCPP_INFO(this->get_logger(), "x=%f y=%f th=%f delta_x=%f delta_y=%f,delta_th=%f", odom_x_, odom_y_, odom_th_, delta_x, delta_y, delta_th);

    // 发布里程计话题，完成TF广播
    odom_publisher(vx, vth);    
}

double OriginbotBase::degToRad(double deg) 
{
    return deg / 180.0 * M_PI;
}

double OriginbotBase::imu_conversion(uint8_t data_high, uint8_t data_low)
{
    short transition_16;
    
    transition_16 = 0;
    transition_16 |= data_high << 8;
    transition_16 |= data_low;

    return transition_16;
}

void OriginbotBase::processAccelerationData(DataFrame &frame)
{
    //RCLCPP_INFO(this->get_logger(), "Process acceleration data");

    imu_data_.acceleration_x = imu_conversion(frame.data[1], frame.data[0]) / 100.0 * degToRad(1.0);
    imu_data_.acceleration_y = imu_conversion(frame.data[3], frame.data[2]) / 100.0 * degToRad(1.0);
    imu_data_.acceleration_z = imu_conversion(frame.data[5], frame.data[4]) / 100.0 * degToRad(1.0);
}

void OriginbotBase::processAngularData(DataFrame &frame)
{
    //RCLCPP_INFO(this->get_logger(), "Process angular data");

    imu_data_.angular_x = imu_conversion(frame.data[1], frame.data[0]) / 100.0 * degToRad(1.0);
    imu_data_.angular_y = imu_conversion(frame.data[3], frame.data[2]) / 100.0 * degToRad(1.0);
    imu_data_.angular_z = imu_conversion(frame.data[5], frame.data[4]) / 100.0 * degToRad(1.0);
}

void OriginbotBase::processEulerData(DataFrame &frame)
{
    //RCLCPP_INFO(this->get_logger(), "Process euler data");

    imu_data_.roll  = imu_conversion(frame.data[1], frame.data[0]) / 100.0 * degToRad(1.0);
    imu_data_.pitch = imu_conversion(frame.data[3], frame.data[2]) / 100.0 * degToRad(1.0);
    imu_data_.yaw   = imu_conversion(frame.data[5], frame.data[4]) / 100.0 * degToRad(1.0);

    if(use_imu_)
        imu_publisher();
}

void OriginbotBase::processSensorData(DataFrame &frame)
{   
    uint16_t dataTemp = frame.data[1];
    robot_status_.tof_distance = (float)((dataTemp << 8) | frame.data[0]) / 1000.0f;
    robot_status_.voice_cmd = (uint16_t)(frame.data[2]);
    uint8_t cycle_time = frame.data[3];

    RCLCPP_INFO(this->get_logger(), "Mcu cycle time = %d ms", cycle_time);
    // robot_status_.battery_voltage = (float)frame.data[0] + ((float)frame.data[1]/100.0);
    // RCLCPP_INFO(this->get_logger(), "Battery Voltage: %0.2f", (float)frame.data[0] + ((float)frame.data[1]/100.0));
}

void OriginbotBase::odom_publisher(float vx, float vth)
{
    auto odom_msg = nav_msgs::msg::Odometry();

    //里程数据计算
    odom_msg.header.frame_id = "odom";
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.pose.pose.position.x = odom_x_;
    odom_msg.pose.pose.position.y = odom_y_;
    odom_msg.pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, odom_th_);
    odom_msg.child_frame_id = "base_footprint";
    odom_msg.pose.pose.orientation.x = q[0];
    odom_msg.pose.pose.orientation.y = q[1];
    odom_msg.pose.pose.orientation.z = q[2];
    odom_msg.pose.pose.orientation.w = q[3];

    const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0,

                                             0, 1e-3, 0, 0, 0, 0,

                                             0, 0, 1e6, 0, 0, 0,
                                             0, 0, 0, 1e6, 0, 0,

                                             0, 0, 0, 0, 1e6, 0,

                                             0, 0, 0, 0, 0, 1e-9};
    const double odom_pose_covariance2[36]= {1e-9,    0,    0,   0,   0,    0,
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9 };

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0.00;
    odom_msg.twist.twist.linear.z = 0.00;

    odom_msg.twist.twist.angular.x = 0.00;
    odom_msg.twist.twist.angular.y = 0.00;
    odom_msg.twist.twist.angular.z = vth;

    const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0,
                                              0, 1e-3, 1e-9, 0, 0, 0,

                                              0, 0, 1e6, 0, 0, 0,

                                              0, 0, 0, 1e6, 0, 0,

                                              0, 0, 0, 0, 1e6, 0,

                                              0, 0, 0, 0, 0, 1e-9};
    const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
                                        0, 1e-3, 1e-9,   0,   0,    0,
                                        0,    0,  1e6,   0,   0,    0,
                                        0,    0,    0, 1e6,   0,    0,
                                        0,    0,    0,   0, 1e6,    0,
                                        0,    0,    0,   0,   0, 1e-9};

    // RCLCPP_INFO(this->get_logger(), "%f %f %f",x,y,th);

    if (vx == 0 && vth == 0)
        memcpy(&odom_msg.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
            memcpy(&odom_msg.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    else
        memcpy(&odom_msg.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
            memcpy(&odom_msg.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));

    // 发布里程计话题
    odom_publisher_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id  = "base_footprint";

    t.transform.translation.x = odom_x_;
    t.transform.translation.y = odom_y_;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = q[0];
    t.transform.rotation.y = q[1];
    t.transform.rotation.z = q[2];
    t.transform.rotation.w = q[3];

    // 广播里程计TF
    tf_broadcaster_->sendTransform(t);
}

void OriginbotBase::imu_publisher()
{
    //RCLCPP_INFO(this->get_logger(), "Imu Data Publish.");

    // 封装IMU的话题消息
    auto imu_msg = sensor_msgs::msg::Imu();

    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = this->get_clock()->now();

    imu_msg.linear_acceleration.x = imu_data_.acceleration_x;
    imu_msg.linear_acceleration.y = imu_data_.acceleration_y;
    imu_msg.linear_acceleration.z = imu_data_.acceleration_z;

    imu_msg.angular_velocity.x = imu_data_.angular_x;
    imu_msg.angular_velocity.y = imu_data_.angular_y;
    imu_msg.angular_velocity.z = imu_data_.angular_z;

    tf2::Quaternion q;
    q.setRPY(imu_data_.roll, imu_data_.pitch, imu_data_.yaw);

    imu_msg.orientation.x = q[0];
    imu_msg.orientation.y = q[1];
    imu_msg.orientation.z = q[2];
    imu_msg.orientation.w = q[3];

    imu_msg.linear_acceleration_covariance = {0.04, 0.00, 0.00, 0.00, 0.04, 0.00, 0.00, 0.00, 0.04};

    imu_msg.angular_velocity_covariance = {0.02, 0.00, 0.00, 0.00, 0.02, 0.00, 0.00, 0.00, 0.02};

    imu_msg.orientation_covariance = {0.0025, 0.0000, 0.0000, 0.0000, 0.0025, 0.0000, 0.0000, 0.0000, 0.0025};

    // 发布IMU话题
    imu_publisher_->publish(imu_msg);
}

bool OriginbotBase::imu_calibration()
{
    DataFrame configFrame;

    // 封装IMU校准命令的数据帧
    configFrame.header = 0x55;
    configFrame.id     = 0x07;
    configFrame.length = 0x06;
    configFrame.data[0]= 0x00;
    configFrame.data[1]= 0x00;
    configFrame.data[2]= 0x00;
    configFrame.data[3]= 0x00;
    configFrame.data[4]= 0xFF;
    configFrame.data[5]= 0xFF;
    configFrame.check = (configFrame.data[0] + configFrame.data[1] + configFrame.data[2] + 
                         configFrame.data[3] + configFrame.data[4] + configFrame.data[5]) & 0xff;
    configFrame.tail   = 0xbb; 

    try
    {
        serial_.write(&configFrame.header, sizeof(configFrame)); //向串口发数据
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); //如果发送数据失败,打印错误信息
    }

    return true;
}

void OriginbotBase::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    DataFrame cmdFrame;
    float leftSpeed = 0.0, rightSpeed = 0.0;

    float x_linear = msg->linear.x; 
    float z_angular = msg->angular.z;

    //差分轮运动学模型求解
    leftSpeed  = x_linear - z_angular * ORIGINBOT_WHEEL_TRACK / 2.0;
    rightSpeed = x_linear + z_angular * ORIGINBOT_WHEEL_TRACK / 2.0;

    // RCLCPP_INFO(this->get_logger(), "leftSpeed = '%f' rightSpeed = '%f'", leftSpeed * 100, rightSpeed * 100);

    if (leftSpeed < 0)
        cmdFrame.data[0] = 0x00;
    else
        cmdFrame.data[0] = 0xff;
    cmdFrame.data[1] = int(abs(leftSpeed) * 1000) & 0xff;         //速度值从m/s变为mm/s
    cmdFrame.data[2] = (int(abs(leftSpeed) * 1000) >> 8) & 0xff;

    if (rightSpeed < 0)
        cmdFrame.data[3] = 0x00;
    else
        cmdFrame.data[3] = 0xff;
    cmdFrame.data[4] = int(abs(rightSpeed) * 1000) & 0xff;        //速度值从m/s变为mm/s
    cmdFrame.data[5] = (int(abs(rightSpeed) * 1000) >> 8) & 0xff;

    cmdFrame.check = (cmdFrame.data[0] + cmdFrame.data[1] + cmdFrame.data[2] + 
                      cmdFrame.data[3] + cmdFrame.data[4] + cmdFrame.data[5]) & 0xff;

    // 封装速度命令的数据帧
    cmdFrame.header = 0x55;
    cmdFrame.id     = 0x01;
    cmdFrame.length = 0x06;
    cmdFrame.tail   = 0xbb;
    try
    {
        serial_.write(&cmdFrame.header, sizeof(cmdFrame)); //向串口发数据
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); //如果发送数据失败,打印错误信息
    }

    // 考虑平稳停车的计数值
    if((fabs(x_linear)>0.0001) || (fabs(z_angular)>0.0001))
        auto_stop_count_ = 0;

    // printf("Frame raw data: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n", 
    //         cmdFrame.header, cmdFrame.id, cmdFrame.length, cmdFrame.data[0], cmdFrame.data[1], cmdFrame.data[2], 
    //         cmdFrame.data[3], cmdFrame.data[4], cmdFrame.data[5], cmdFrame.check, cmdFrame.tail);
}

bool OriginbotBase::voice_control(uint16_t req)
{
    DataFrame configFrame;

    // 封装蜂鸣器指令的数据帧
    configFrame.header = 0x55;
    configFrame.id     = 0x07;
    configFrame.length = 0x06;
    configFrame.data[0]= 0x00;
    configFrame.data[1]= 0x00;
    configFrame.data[2]= 0xFF;

    configFrame.data[3]= uint8_t(req);

    configFrame.data[4]= 0x00;
    configFrame.data[5]= 0x00;
    configFrame.check = (configFrame.data[0] + configFrame.data[1] + configFrame.data[2] + 
                         configFrame.data[3] + configFrame.data[4] + configFrame.data[5]) & 0xff;
    configFrame.tail   = 0xbb; 

    try
    {
        serial_.write(&configFrame.header, sizeof(configFrame)); //向串口发数据
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); //如果发送数据失败,打印错误信息
    }

    return true;
}

void OriginbotBase::voice_callback(const std::shared_ptr<originbot_msgs::srv::OriginbotVoice::Request>  request,
                                          std::shared_ptr<originbot_msgs::srv::OriginbotVoice::Response> response)
{
    robot_status_.voice_on = bool(request->req);

    if(voice_control(request->req))
    {
        RCLCPP_INFO(this->get_logger(), "Set Voice state to %d", robot_status_.voice_on);
        response->result = true;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Set Voice state error [%d]", robot_status_.voice_on);
        response->result = false;        
    }
}

bool OriginbotBase::shovel_control(uint8_t angle)
{
    DataFrame configFrame;

    // 封装控制Servo指令的数据帧
    configFrame.header = 0x55;
    configFrame.id     = 0x07;
    configFrame.length = 0x06;
    configFrame.data[0]= 0xFF;

    configFrame.data[1]= angle;

    configFrame.data[2]= 0x00;
    configFrame.data[3]= 0x00;
    configFrame.data[4]= 0x00;
    configFrame.data[5]= 0x00;
    configFrame.check = (configFrame.data[0] + configFrame.data[1] + configFrame.data[2] + 
                         configFrame.data[3] + configFrame.data[4] + configFrame.data[5]) & 0xff;
    configFrame.tail   = 0xbb; 

    try
    {
        serial_.write(&configFrame.header, sizeof(configFrame)); //向串口发数据
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); //如果发送数据失败,打印错误信息
    }

    return true;
}

void OriginbotBase::shovel_callback(const std::shared_ptr<originbot_msgs::srv::OriginbotServo::Request>  request,
                                       std::shared_ptr<originbot_msgs::srv::OriginbotServo::Response> response)
{
    robot_status_.servo_on = bool(request->angle);

    if(shovel_control(robot_status_.servo_on))
    {
        RCLCPP_INFO(this->get_logger(), "Set Servo state to %d", robot_status_.servo_on);
        response->result = true;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Set Servo state error [%d]", robot_status_.servo_on);
        response->result = false;        
    }
}                              

void OriginbotBase::pid_callback(const std::shared_ptr<originbot_msgs::srv::OriginbotPID::Request>  request,
                                       std::shared_ptr<originbot_msgs::srv::OriginbotPID::Response> response)
{
    short motor_p = (short)(request->p);
    short motor_i = (short)(request->i);
    short motor_d = (short)(request->d);

    DataFrame pidFrame;

    // 封装PID参数的数据帧
    pidFrame.header = 0x55;
    pidFrame.id     = 0x08;
    pidFrame.length = 0x06;
    pidFrame.data[0]= motor_p & 0xFF;
    pidFrame.data[1]= (motor_p>>8) & 0xFF;
    pidFrame.data[2]= motor_i & 0xFF;;
    pidFrame.data[3]= (motor_i>>8) & 0xFF;
    pidFrame.data[4]= motor_d & 0xFF;;
    pidFrame.data[5]= (motor_d>>8) & 0xFF;
    pidFrame.check = (pidFrame.data[0] + pidFrame.data[1] + pidFrame.data[2] + 
                      pidFrame.data[3] + pidFrame.data[4] + pidFrame.data[5]) & 0xff;
    pidFrame.tail   = 0xbb; 

    try
    {
        serial_.write(&pidFrame.header, sizeof(pidFrame)); //向串口发数据
    }

    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); //如果发送数据失败,打印错误信息
    }

    RCLCPP_INFO(this->get_logger(), "Set motor pid parameters to [%0.4f %0.4f %0.4f]", request->p, request->i, request->d);

    // printf("Frame raw data: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n", 
    //         pidFrame.header, pidFrame.id, pidFrame.length, pidFrame.data[0], pidFrame.data[1], pidFrame.data[2], 
    //         pidFrame.data[3], pidFrame.data[4], pidFrame.data[5], pidFrame.check, pidFrame.tail);

    response->result = true;
}                       

void OriginbotBase::timer_100ms_callback()
{
    // 是否开启自动停车，且出发自动停车的条件
    if(auto_stop_on_ && auto_stop_count_<20)
    {
        auto_stop_count_ ++;

         // 0.5之内没有收到指令的话，就自动停车
        if(auto_stop_count_ > 5)
        {
            auto_stop_count_ = 255;

            DataFrame cmdFrame;
            cmdFrame.data[0] = 0x00;
            cmdFrame.data[1] = 0x00;         
            cmdFrame.data[2] = 0x00;
            cmdFrame.data[3] = 0x00;
            cmdFrame.data[4] = 0x00; 
            cmdFrame.data[5] = 0x00;
            cmdFrame.check = (cmdFrame.data[0] + cmdFrame.data[1] + cmdFrame.data[2] + 
                            cmdFrame.data[3] + cmdFrame.data[4] + cmdFrame.data[5]) & 0xff;

            // 封装速度命令的数据帧
            cmdFrame.header = 0x55;
            cmdFrame.id     = 0x01;
            cmdFrame.length = 0x06;
            cmdFrame.tail   = 0xbb;
            try
            {
                serial_.write(&cmdFrame.header, sizeof(cmdFrame)); //向串口发数据
                RCLCPP_DEBUG(this->get_logger(), "Execute auto stop");
            }

            catch (serial::IOException &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); //如果发送数据失败,打印错误信息
            }           
        }
    }

    // 发布机器人的状态信息
    originbot_msgs::msg::OriginbotStatus status_msg;

    status_msg.battery_voltage = robot_status_.battery_voltage;
    status_msg.tof_distance = robot_status_.tof_distance;
    status_msg.voice_cmd = robot_status_.voice_cmd;
    status_msg.voice_on = robot_status_.voice_on;
    status_msg.servo_on = robot_status_.servo_on;

    status_publisher_->publish(status_msg);
}

void sigintHandler(int sig)
{
    sig = sig;
    
    printf("OriginBot shutdown...\n");

    serial::Serial serial;
    serial.setPort("/dev/ttyS3");                                   //选择要开启的串口号
    serial.setBaudrate(115200);                                     //设置波特率
    serial::Timeout timeOut = serial::Timeout::simpleTimeout(2000); //超时等待
    serial.setTimeout(timeOut);                                     
    serial.open();                                                  //开启串口

    // 如果串口打开，则驱动读取数据的线程
    if (serial.isOpen())
    {
        // 程序退出时自动停车
        DataFrame cmdFrame;
        cmdFrame.data[0] = 0x00;
        cmdFrame.data[1] = 0x00;         
        cmdFrame.data[2] = 0x00;
        cmdFrame.data[3] = 0x00;
        cmdFrame.data[4] = 0x00; 
        cmdFrame.data[5] = 0x00;
        cmdFrame.check = (cmdFrame.data[0] + cmdFrame.data[1] + cmdFrame.data[2] + 
                        cmdFrame.data[3] + cmdFrame.data[4] + cmdFrame.data[5]) & 0xff;

        // 封装速度命令的数据帧
        cmdFrame.header = 0x55;
        cmdFrame.id     = 0x01;
        cmdFrame.length = 0x06;
        cmdFrame.tail   = 0xbb;

        try
        {
            serial.write(&cmdFrame.header, sizeof(cmdFrame)); //向串口发数据
            printf("Execute auto stop\n");
        }
        catch (serial::IOException &e)
        {
            printf("Unable to send data through serial port\n"); //如果发送数据失败,打印错误信息
        }           
    }

    // 关闭ROS2接口，清除资源
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    // 初始化ROS节点
    rclcpp::init(argc, argv);

    // 创建信号处理函数
    signal(SIGINT, sigintHandler);

    // 创建机器人底盘类，通过spin不断查询订阅话题
    rclcpp::spin(std::make_shared<OriginbotBase>("originbot_base"));
    
    // 关闭ROS2接口，清除资源
    rclcpp::shutdown();

    return 0;
}