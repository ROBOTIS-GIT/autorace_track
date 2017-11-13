/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#ifndef TURTLEBOT3_MOTOR_DRIVER_H_
#define TURTLEBOT3_MOTOR_DRIVER_H_

#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series)
// #define ADDR_X_ACCELERATION_LIMIT       40
// #define ADDR_X_VELOCITY_LIMIT           44
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_PROFILE_ACCELERATION     108
#define ADDR_X_PROFILE_VELOCITY         112
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_VELOCITY         128
#define ADDR_X_PRESENT_POSITION         132

// Limit values (XM430-W210-T)
#define LIMIT_X_MAX_VELOCITY            240

// Data Byte Length
#define LEN_X_ACCELERATION_LIMIT        4
#define LEN_X_VELOCITY_LIMIT            4
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_PROFILE_VELOCITY          4
#define LEN_X_PROFILE_ACCELERATION      4
#define LEN_X_GOAL_POSITION             4
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_VELOCITY          4
#define LEN_X_PRESENT_POSITION          4

#define PROTOCOL_VERSION                2.0     // Dynamixel protocol version 2.0

#define MOTOR_ID                        1       // ID of thr motor
#define BAUDRATE                        57600   // Baudrate of Dynamixel
#define DEVICENAME                      "1"     // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define PROFILE_VELOCITY_VALUE          200
#define PROFILE_ACCELERATION_VALUE      30

class MotorDriver
{
 public:
  MotorDriver();
  ~MotorDriver();
  bool init(void);
  void closeDynamixel(void);
  void setAccelerationLimit(uint8_t id, int64_t acceleration_limit_value);
  void setVelocityLimit(uint8_t id, int64_t velocity_limit_value);
  void setTorque(uint8_t id, bool onoff);
  void setProfileAcceleration(uint8_t id, int64_t profile_acceleration_value);
  void setProfileVelocity(uint8_t id, int64_t profile_velocity_value);
  void controlPosition(uint8_t id, int64_t position_value);
  // bool readEncoder(int32_t &left_value, int32_t &right_value);
  // bool speedControl(int64_t left_wheel_value, int64_t right_wheel_value);

 private:
  uint32_t baudrate_;
  float  protocol_version_;
  uint8_t motor_id_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  // dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
  // dynamixel::GroupSyncRead *groupSyncReadEncoder_;
};

#endif // TURTLEBOT3_MOTOR_DRIVER_H_
