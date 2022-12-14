// INCLUDE LIBS
#include "Wire.h"             // I2C
#include "VL53L0X.h"          // TOF sensors
#include "MPU9250.h"          // MPU9250
#include <SoftwareSerial.h>   // 软串口
#include "eeprom_utils.h"
#include "math.h"

#define LEFT 1
#define RIGHT 2
#define YAW 3
#define DEG2RAD PI/180.0
//电机一圈单个信号13个脉冲，减速比1:45, 电机一圈共有13*45=585个计数
#define WHEEL_CIRCUMFERENCE_MM (230.0)
#define SPD_MM_PER_COUNT (WHEEL_CIRCUMFERENCE_MM / 635.0)

#define TrackWidth 124.0  // 轮距

const uint8_t Motor1_PinA = 5;
const uint8_t Motor1_PinB = 6;
const uint8_t Motor2_PinA = 9;
const uint8_t Motor2_PinB = 10;
const uint8_t Encoder1_PinA = 2;  //中断1
const uint8_t Encoder1_PinB = 4;
const uint8_t Encoder2_PinA = 3;  //中断2
const uint8_t Encoder2_PinB = 7;

const int kp_add = 5;
const int kd_add = 2;

int sound_ctrl_vel = 300;
int sound_ctrl_diff = 60;     // 差速量
int sound_ctrl_add = 100;      //加减速量

VL53L0X tof;
MPU9250 mpu; 
SoftwareSerial mySerial(12, 13); // RX, TX
byte s_data[9] = {0};

volatile long encoder1_count = 0;
volatile long encoder2_count = 0;
long encoder1_count_last = 0;
long encoder2_count_last = 0;
int speed_left = 0;    //mm/s
int speed_right = 0;
int speed_left_last = 0;
int speed_right_last = 0;
int yaw_last = 0;      // 上次的航向角 * 10

volatile int tof_distance = 0;
volatile uint8_t rx_data_type = 0;

volatile int ctrl_speed_left = 0; //mm/s
volatile int ctrl_speed_right = 0; //mm/s
volatile int kp = 80;
volatile int ki = 0;
volatile int kd = 10;
volatile byte sound_cmd_from_mcu = 0;
volatile byte sound_cmd_from_x3pi = 0;
volatile bool calibration_imu_flag = false;
volatile bool shovel_enable = false;
volatile byte shovel_pwm = 0;

unsigned long last_time_stamp = 0;
unsigned long last_time = 0;
byte cycle_time = 0;
byte train_mode_flag = 0;

float train_yaw = 0.0;
long train_encoder = 0;



void setup() {
  pinMode(Motor1_PinA, OUTPUT);
  pinMode(Motor1_PinB, OUTPUT);
  pinMode(Motor2_PinA, OUTPUT);
  pinMode(Motor2_PinB, OUTPUT);

  pinMode(Encoder1_PinA, INPUT_PULLUP);
  pinMode(Encoder1_PinB, INPUT);
  pinMode(Encoder2_PinA, INPUT_PULLUP);
  pinMode(Encoder2_PinB, INPUT);  

  attachInterrupt(1, read_encoder2, FALLING);  //digitalPinToInterrupt(Encoder1_PinA)
  attachInterrupt(0, read_encoder1, FALLING); 

  Serial.begin(115200);
  mySerial.begin(57600);
  Wire.begin();
  delay(200);

  mpu.setup(0x68); 
  // load mpu9250 param
  loadCalibration();  
  mpu.setMagneticDeclination(-6.26);
  //default value is 1. Generally 10-20 is good for stable yaw estimation
  mpu.setFilterIterations(1); 

  tof.setTimeout(500);
  tof.init();
  tof.startContinuous();

  // load PID param
  // savePidParam();
  readPidParam();

  s_data[0] = 0xaa;
  s_data[1] = 0x55;
  s_data[7] = 0x55;
  s_data[8] = 0xaa;
  yaw_last = (int)(mpu.getYaw() * 10.0);

  interrupts();    // 开启中断
}

// the loop function runs over and over again forever
void loop() {
  mpu.update();
  process_speed();  
  send_imu_speed_data();
  velocity_control();

  unsigned long ts = millis() - last_time;
  if (ts > 100) {     // 100ms定时器
    timer_loop();
    last_time = millis();
  }
}

void timer_loop() {  
  if (mySerial.available()) {
    int data = mySerial.read();
    switch (data) {
      case 0x21:
        ctrl_speed_left = sound_ctrl_vel;
        ctrl_speed_right = sound_ctrl_vel;
        break; 
      case 0x22:
        ctrl_speed_left = -sound_ctrl_vel;
        ctrl_speed_right = -sound_ctrl_vel;
        break; 
      case 0x23:
        if (ctrl_speed_left == 0 && ctrl_speed_right == 0) {
          ctrl_speed_left = -sound_ctrl_vel;
          ctrl_speed_right = sound_ctrl_vel;
        } else {
          ctrl_speed_left -= sound_ctrl_diff;
          ctrl_speed_right += sound_ctrl_diff;
        }
        break; 
      case 0x24:
        if (ctrl_speed_left == 0 && ctrl_speed_right == 0) {
          ctrl_speed_left = sound_ctrl_vel;
          ctrl_speed_right = -sound_ctrl_vel;
        } else {
          ctrl_speed_left += sound_ctrl_diff;
          ctrl_speed_right -= sound_ctrl_diff;
        }
        break; 
      case 0x25:
        ctrl_speed_left += sound_ctrl_add;
        ctrl_speed_right += sound_ctrl_add; 
        break;    
      case 0x26:
        ctrl_speed_left -= sound_ctrl_add;
        ctrl_speed_right -= sound_ctrl_add;
        break; 
      case 0x27:
        train_mode_flag = 0x27;
        break; 
      case 0x28:
        train_mode_flag = 0x28;
        break; 
      case 0x29:
        ctrl_speed_left = 0;
        ctrl_speed_right = 0;  
        train_mode_flag = 0;    // 复位
        break; 
      case 0x2a:
        train_mode_flag = 0x2a;
        break; 
      case 0x2b:
        train_mode_flag = 0x2b;
        break; 
      case 0x2c:
        train_mode_flag = 0x2c;
        break; 
      case 0x2d:
        train_mode_flag = 0x2d;
        break; 
      case 0x31:
        kp += kp_add;
        break; 
      case 0x32:
        kp -= kp_add;
        break; 
      case 0x33:
        kd += kd_add;
        break; 
      case 0x34:
        kd -= kd_add;
        break; 
      case 0x35:
        s_data[2] = 31;
        s_data[3] = kp & 0xff;
        s_data[4] = (kp >> 8) & 0xff;
        mySerial.write(s_data, 9);
        s_data[4] = 0x00;
        break;
      case 0x36:
        s_data[2] = 32;
        s_data[3] = kd & 0xff;
        s_data[4] = (kd >> 8) & 0xff;
        mySerial.write(s_data, 9);
        s_data[4] = 0x00;
        break;
      case 0x39:
        savePidParam();
        break;
      default:
        sound_cmd_from_mcu = data;
        break; 
    }   
  }

  // 10HZ cycle
  send_sensor_data();
  process_tof();
  train_mode_fun();

}

void serialEvent() {  
	while (Serial.available()) {
    if (Serial.read() == 0x55) {
      delay(1);     //把后面字节都接收近来
      if (Serial.available()) {
        rx_data_type = Serial.read(); 
        if ((rx_data_type == 0x01) || (rx_data_type == 0x07) || (rx_data_type == 0x08)) {
          if (Serial.available() && ((uint8_t)(Serial.read()) == 0x06)) {
            while (Serial.available() < 7) {};
            byte serial_buffer[8] = {0};        // TODO： crc校验
            Serial.readBytes(serial_buffer, 8);
            parse_cmd_data(serial_buffer, rx_data_type);         
          } 
        }       
      }
    } 
  }
}

void velocity_control() {
  int diff_pwm = 0;
  int left_pwm = do_pid(ctrl_speed_left, speed_left, LEFT);
  int right_pwm = do_pid(ctrl_speed_right, speed_right, RIGHT);
  if (ctrl_speed_left == ctrl_speed_right && ctrl_speed_left != 0) {
    diff_pwm = do_pid(yaw_last, (int)(mpu.getYaw() * 10.0), YAW);
    left_pwm += diff_pwm;
    right_pwm -= diff_pwm;
  } else {
    yaw_last = (int)(mpu.getYaw() * 10.0);
  }
  if (left_pwm < 0) {
    analogWrite(Motor1_PinA, map(abs(left_pwm), 0, 30000, 0, 255));
    analogWrite(Motor1_PinB, 0);
  } else {
    analogWrite(Motor1_PinA, 0);
    analogWrite(Motor1_PinB, map(abs(left_pwm), 0, 30000, 0, 255));
  }
  if (right_pwm > 0) {
    analogWrite(Motor2_PinA, map(abs(right_pwm), 0, 30000, 0, 255));
    analogWrite(Motor2_PinB, 0);
  } else {
    analogWrite(Motor2_PinA, 0);
    analogWrite(Motor2_PinB, map(abs(right_pwm), 0, 30000, 0, 255));
  }
}

int do_pid(int target, int feedback, uint8_t l_r) {
  static int last_error_left = 0;
  static int last_error_right = 0;
  long value = 0;
  long error = target - feedback;
  if (l_r == LEFT) {
    value = kp * error + kd * (error - last_error_left);
    last_error_left = error;
    value = constrain(value, -30000, 30000);
    return value;
  }
  if (l_r == RIGHT) {
    value = kp * error + kd * (error - last_error_right);
    last_error_right = error;
    value = constrain(value, -30000, 30000);
    return value;
  }
  if (l_r == YAW) {
    if (error > 1800) {
      error -= 3600;      
    } else if (error < -1800) {
      error += 3600;
    }
    value = 80 * error;
    value = constrain(value, -8000, 8000);
    return value;
  }
  return 0;
}

//0x01=control  0x07=外设  0x08=PID
void parse_cmd_data(byte *serial_data_buffer, int rx_data_type) {

  word left_speed=0, right_speed=0;
  uint8_t sound_request_flag=0;
  word tmp = 0;

  switch (rx_data_type) {
    case 0x01:
      left_speed = serial_data_buffer[2];
      left_speed = (left_speed << 8) | serial_data_buffer[1];
      right_speed = serial_data_buffer[5];
      right_speed = (right_speed << 8) | serial_data_buffer[4];
      if (serial_data_buffer[0] == 0) {
        ctrl_speed_left = -(int)(left_speed);
      } else {
        ctrl_speed_left = (int)(left_speed);
      }
      if (serial_data_buffer[3] == 0) {
        ctrl_speed_right = -(int)(right_speed);
      } else {
        ctrl_speed_right = (int)(right_speed);
      }
      break;     
    case 0x07:
      shovel_enable = bool(serial_data_buffer[0]);
      shovel_pwm = serial_data_buffer[1];
      sound_request_flag = serial_data_buffer[2];
      sound_cmd_from_x3pi = serial_data_buffer[3];
      if (sound_request_flag != 0) {
        s_data[2] = 61;
        s_data[3] = sound_cmd_from_x3pi;
        mySerial.write(s_data, 9);        
      }
      calibration_imu_flag = bool(serial_data_buffer[4]);
      break; 
    case 0x08:
      tmp = serial_data_buffer[1];
      kp = (tmp << 8) | serial_data_buffer[0];
      tmp = serial_data_buffer[3];
      ki = (tmp << 8) | serial_data_buffer[2];
      tmp = serial_data_buffer[5];
      kd = (tmp << 8) | serial_data_buffer[4];
      savePidParam();
      break; 
  }
}

void send_imu_speed_data() {
  word tmp = 0;
  int tmp2 = 0;
  byte send_data_buffer[11] = {0};

  send_data_buffer[0] = 0x55;
  send_data_buffer[2] = 0x06;
  send_data_buffer[10] = 0xBB;

  // send speed
  send_data_buffer[1] = 0x02;
  if (speed_left >= 0) {
    send_data_buffer[3] = 0xff;
  } else {
    send_data_buffer[3] = 0x00;
  }
  if (speed_right >= 0) {
    send_data_buffer[6] = 0xff;
  } else {
    send_data_buffer[6] = 0x00;
  }
  tmp = (word)abs(speed_left);
  send_data_buffer[4] = tmp & 0xff;
  send_data_buffer[5] = (tmp >> 8) & 0xff;
  tmp = (word)abs(speed_right);
  send_data_buffer[7] = tmp & 0xff;
  send_data_buffer[8] = (tmp >> 8) & 0xff;
  send_data_buffer[9]  = 0;
  for (byte i = 3; i < 9; ++i) {
    send_data_buffer[9] += send_data_buffer[i];
  }
  send_data_buffer[9] = send_data_buffer[9] & 0xff;
  Serial.write(send_data_buffer, 11);

  // imu data
  send_data_buffer[1] = 0x03;
  tmp2 =(int)(mpu.getAccX() * 100.0);
  send_data_buffer[3] = tmp2 & 0xff;
  send_data_buffer[4] = (tmp2 >> 8) & 0xff;
  tmp2 =(int)(mpu.getAccY() * 100.0);
  send_data_buffer[5] = tmp2 & 0xff;
  send_data_buffer[6] = (tmp2 >> 8) & 0xff;
  tmp2 =(int)(mpu.getAccZ() * 100.0);
  send_data_buffer[7] = tmp2 & 0xff;
  send_data_buffer[8] = (tmp2 >> 8) & 0xff;
  send_data_buffer[9]  = 0;
  for (byte i = 3; i < 9; ++i) {
    send_data_buffer[9] += send_data_buffer[i];
  }
  send_data_buffer[9] = send_data_buffer[9] & 0xff;
  Serial.write(send_data_buffer, 11);
  
  send_data_buffer[1] = 0x04;
  tmp2 =(int)(mpu.getGyroX() * 100.0);
  send_data_buffer[3] = tmp2 & 0xff;
  send_data_buffer[4] = (tmp2 >> 8) & 0xff;
  tmp2 =(int)(mpu.getGyroY() * 100.0);
  send_data_buffer[5] = tmp2 & 0xff;
  send_data_buffer[6] = (tmp2 >> 8) & 0xff;
  tmp2 =(int)(mpu.getGyroZ() * 100.0);
  send_data_buffer[7] = tmp2 & 0xff;
  send_data_buffer[8] = (tmp2 >> 8) & 0xff;
  send_data_buffer[9] = 0;
  for (byte i = 3; i < 9; ++i) {
    send_data_buffer[9] += send_data_buffer[i];
  }
  send_data_buffer[9] = send_data_buffer[9] & 0xff;
  Serial.write(send_data_buffer, 11);
  
  send_data_buffer[1] = 0x05;
  tmp2 =(int)(mpu.getRoll() * 100.0);
  send_data_buffer[3] = tmp2 & 0xff;
  send_data_buffer[4] = (tmp2 >> 8) & 0xff;
  tmp2 =(int)(mpu.getPitch() * 100.0);
  send_data_buffer[5] = tmp2 & 0xff;
  send_data_buffer[6] = (tmp2 >> 8) & 0xff;
  tmp2 =(int)(mpu.getYaw() * 100.0);
  send_data_buffer[7] = tmp2 & 0xff;
  send_data_buffer[8] = (tmp2 >> 8) & 0xff;
  send_data_buffer[9] = 0;
  for (byte i = 3; i < 9; ++i) {
    send_data_buffer[9] += send_data_buffer[i];
  }
  send_data_buffer[9] = send_data_buffer[9] & 0xff;
  Serial.write(send_data_buffer, 11);
}

void send_sensor_data() {
  int tmp = 0;
  byte send_data_buffer[11] = {0};

  send_data_buffer[0] = 0x55;
  send_data_buffer[1] = 0x06;
  send_data_buffer[2] = 0x06;
  send_data_buffer[10] = 0xBB;

  // Send TOF distance
  send_data_buffer[3] = tof_distance & 0xff;
  send_data_buffer[4] = (tof_distance >> 8) & 0xff;

  // send sound control
  send_data_buffer[5] = sound_cmd_from_mcu;
  send_data_buffer[6] = cycle_time;  
  send_data_buffer[9] = 0;
  for (byte i = 3; i < 9; ++i) {
    send_data_buffer[9] += send_data_buffer[i];
  }
  send_data_buffer[9] = send_data_buffer[9] & 0xff;
  Serial.write(send_data_buffer, 11);
}

void read_encoder1() {
  if (digitalRead(Encoder1_PinA) == LOW)
  {
    if (digitalRead(Encoder1_PinB) == LOW) {
      encoder1_count --; 
    } else if (digitalRead(Encoder1_PinB) == HIGH) { 
      encoder1_count ++; 
    }
  }
}

void read_encoder2() {
  if (digitalRead(Encoder2_PinA) == LOW)
  {
    if (digitalRead(Encoder2_PinB) == LOW) {
      encoder2_count ++; 
    } else if (digitalRead(Encoder2_PinB) == HIGH) { 
      encoder2_count --; 
    }
  }
}

void process_speed() {
  float ts = (float)(micros() - last_time_stamp) / 1000000.0f;
  cycle_time = byte(ts * 1000);
  speed_left = (float)(encoder1_count - encoder1_count_last) * SPD_MM_PER_COUNT / ts;
  speed_right = (float)(encoder2_count - encoder2_count_last) * SPD_MM_PER_COUNT / ts;
  speed_left = int(speed_left_last * 0.9 + speed_left * 0.1);
  speed_right = int(speed_right_last * 0.9 + speed_right * 0.1);
  speed_left_last = speed_left;
  speed_right_last = speed_right;
  encoder1_count_last = encoder1_count;
  encoder2_count_last = encoder2_count;
  last_time_stamp = micros();
}

float process_tof() {
  tof_distance = tof.readRangeContinuousMillimeters();
  // if (tof.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  // Serial.print("TOF = ");
  // Serial.println(tof_distance);
}

void train_mode_fun() {
  static bool first_in_flag = true;
  // Serial.println(train_mode_flag);

  switch (train_mode_flag) {
    case 0x0:
      first_in_flag = true;
      break;
    case 0x27:    // 左传90
      if (first_in_flag) {
        train_yaw = mpu.getYaw();
        first_in_flag = false;
      }
      ctrl_speed_left = -sound_ctrl_vel;
      ctrl_speed_right = sound_ctrl_vel;
      if (cos((mpu.getYaw()-train_yaw) * DEG2RAD) < 0) {
        ctrl_speed_left = ctrl_speed_right = 0;
        first_in_flag = true;
        train_mode_flag = 0;
      }
      break; 
    case 0x28:
      if (first_in_flag) {
        train_yaw = mpu.getYaw();
        first_in_flag = false;
      }
      ctrl_speed_left = sound_ctrl_vel;
      ctrl_speed_right = -sound_ctrl_vel;
      if (cos((mpu.getYaw()-train_yaw) * DEG2RAD) < 0) {
        ctrl_speed_left = ctrl_speed_right = 0;
        first_in_flag = true;
        train_mode_flag = 0;
      }
      break; 
    case 0x2a:
      if (first_in_flag) {
        train_yaw = mpu.getYaw();        
        first_in_flag = false;
      }
      ctrl_speed_left = sound_ctrl_vel;
      ctrl_speed_right = -sound_ctrl_vel;
      if (cos((mpu.getYaw()-train_yaw) * DEG2RAD) < -0.98) {
        ctrl_speed_left = ctrl_speed_right = 0;
        first_in_flag = true;
        train_mode_flag = 0;
      }
      break; 
    case 0x2b:
      if (first_in_flag) {
        train_yaw = mpu.getYaw();  
        first_in_flag = false;
      }
      ctrl_speed_left = sound_ctrl_vel;
      ctrl_speed_right = 0;
      if (cos((mpu.getYaw()-train_yaw) * DEG2RAD) < -0.98) {
        ctrl_speed_left = ctrl_speed_right = 0;
        first_in_flag = true;
        train_mode_flag = 0;
      }
      break; 
    case 0x2c:
      if (first_in_flag) {
        train_yaw = mpu.getYaw(); 
        first_in_flag = false;
      }
      ctrl_speed_left = 0;
      ctrl_speed_right = sound_ctrl_vel;
      if (cos((mpu.getYaw()-train_yaw) * DEG2RAD) < -0.98) {
        ctrl_speed_left = ctrl_speed_right = 0;
        first_in_flag = true;
        train_mode_flag = 0;
      }
      break;  
    default:
      break;
  }
}



