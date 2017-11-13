/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
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

/* Authors: Leon Jung */

#include "motor_driver.h"

MotorDriver::MotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  motor_id_(MOTOR_ID)
{
}

MotorDriver::~MotorDriver()
{
  closeDynamixel();
}

bool MotorDriver::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort())
  {
    #ifdef DEBUG
    sprintf(log_msg, "Port is Opened");
    nh.loginfo(log_msg);
    #endif
  }
  else
  {
    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_))
  {
    #ifdef DEBUG
    sprintf(log_msg, "Baudrate is set");
    nh.loginfo(log_msg);
    #endif
  }
  else
  {
    return false;
  }

  // setVelocityLimit(motor_id_, Pw_, PROFILE_ACCELERATION_VALUE);

  // Enable Dynamixel Torque
  setTorque(motor_id_, true);

  setProfileVelocity(motor_id_, PROFILE_VELOCITY_VALUE);
  setProfileAcceleration(motor_id_, PROFILE_ACCELERATION_VALUE);

  // controlPosition(motor_id_, 2048);
  // groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  // groupSyncReadEncoder_   = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  return true;
}

void MotorDriver::setAccelerationLimit(uint8_t id, int64_t acceleration_limit_value)
{
  uint8_t dxl_error = 0;
  int8_t dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, motor_id_, ADDR_X_PROFILE_ACCELERATION, acceleration_limit_value, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->printRxPacketError(dxl_error);
  }
}

void MotorDriver::setVelocityLimit(uint8_t id, int64_t velocity_limit_value)
{
  uint8_t dxl_error = 0;
  int8_t dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, motor_id_, ADDR_X_PROFILE_VELOCITY, velocity_limit_value, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->printRxPacketError(dxl_error);
  }
}


void MotorDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->printRxPacketError(dxl_error);
  }
}

void MotorDriver::setProfileAcceleration(uint8_t id, int64_t profile_acceleration_value)
{
  uint8_t dxl_error = 0;
  int8_t dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, motor_id_, ADDR_X_PROFILE_ACCELERATION, profile_acceleration_value, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->printRxPacketError(dxl_error);
  }
}

void MotorDriver::setProfileVelocity(uint8_t id, int64_t profile_velocity_value)
{
  uint8_t dxl_error = 0;
  int8_t dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, motor_id_, ADDR_X_PROFILE_VELOCITY, profile_velocity_value, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->printRxPacketError(dxl_error);
  }
}

void MotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(motor_id_, false);

  // Close port
  portHandler_->closePort();
}

// bool MotorDriver::readEncoder(int32_t &left_value, int32_t &right_value)
// {
//   int dxl_comm_result = COMM_TX_FAIL;              // Communication result
//   bool dxl_addparam_result = false;                // addParam result
//   bool dxl_getdata_result = false;                 // GetParam result
//
//   // Set parameter
//   dxl_addparam_result = groupSyncReadEncoder_->addParam(motor_id_);
//   if (dxl_addparam_result != true)
//     return false;
//
//   dxl_addparam_result = groupSyncReadEncoder_->addParam(right_wheel_id_);
//   if (dxl_addparam_result != true)
//     return false;
//
//   // Syncread present position
//   dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
//   if (dxl_comm_result != COMM_SUCCESS)
//     packetHandler_->printTxRxResult(dxl_comm_result);
//
//   // Check if groupSyncRead data of Dynamixels are available
//   dxl_getdata_result = groupSyncReadEncoder_->isAvailable(motor_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
//   if (dxl_getdata_result != true)
//     return false;
//
//   dxl_getdata_result = groupSyncReadEncoder_->isAvailable(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
//   if (dxl_getdata_result != true)
//     return false;
//
//   // Get data
//   left_value  = groupSyncReadEncoder_->getData(motor_id_,  ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
//   right_value = groupSyncReadEncoder_->getData(right_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
//
//   groupSyncReadEncoder_->clearParam();
//   return true;
// }
//
void MotorDriver::controlPosition(uint8_t id, int64_t position_value)
{
  uint8_t dxl_error = 0;
  int8_t dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_X_GOAL_POSITION, position_value, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->printRxPacketError(dxl_error);
  }
}
