/* RBiz Challenge TurtleBot3 Auto Race - Level Crossing


|-----|
||---||----|-------|-------|-------|-------|
||DXL||    |   W   |   R   |   W   |   R   |
||---||----|-------|-------|-------|-------|
|     |
|     |
|     |
|     |            |---------|            |---------|            |---------|
|-----|            | O-----0 |            | O-----0 |            | O-----0 |
  | |              |   DMS   |            |   DMS   |            |   DMS   |
  | |              | Sensor1 |            | Sensor2 |            | Sensor3 |
  | |              |         |            |         |            |         |
=======------------===========------------===========------------===========

 created 1 October 2017
 by ROBOTIS CO,.LTD.

 author : Leon Ryuwoon Jung
 */

#include <OLLO.h>
#include <DynamixelSDK.h>

#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_GOAL_VELOCITY            132

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        57600
#define DEVICENAME                      "1"                 // Check which port is being used on your controller
                                                            // ex) Serial1: "1"   Serial2: "2"   Serial3: "3"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_POSITION_VALUE_CLOSED       100                 // Dynamixel will rotate between this value
#define DXL_POSITION_VALUE_OPENED       4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

dynamixel::PortHandler *portHandler_;
dynamixel::PacketHandler *packetHandler_;
uint32_t baudrate_;
float  protocol_version_;

uint8_t dxl_error = 0;                                      // Dynamixel error
int32_t dxl_present_position = 0;                           // Present position

OLLO myOLLO;

enum State { WAITING_FOR_ENTER, ENTERED, MUST_STOP, IS_STOPPED } state;
enum Level { DXL_CLOSED, DXL_OPENED };

double stopwatch_start_time_ = 0.0;
bool is_started[3] = { false, false, false };
bool is_able_to_pass_ = true;


bool fnSetDXLTorque(uint8_t id, bool onoff)
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

bool fnInitDXL(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort())
  {
    Serial.print("Port is Opened");
  }
  else
  {
    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_))
  {
    Serial.print("Baudrate is set");
  }
  else
  {
    return false;
  }

  // Enable Dynamixel Torque
  fnSetDXLTorque(DXL_ID, true);

  return true;
}

void fnCloseDXL(void)
{
  // Disable Dynamixel Torque
  fnSetDXLTorque(DXL_ID, false);


  // Close port
  portHandler_->closePort();
}

void fnWriteDXL(Level level_status)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (level_status == DXL_CLOSED)
  {
    // Write goal position to DXL_CLOSED level status
    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, DXL_ID, ADDR_X_GOAL_POSITION, DXL_POSITION_VALUE_CLOSED, &dxl_error);
  }
  else
  {
    // Write goal position to DXL_OPENED level status
    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, DXL_ID, ADDR_X_GOAL_POSITION, DXL_POSITION_VALUE_OPENED, &dxl_error);
  }

  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.print(packetHandler_->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    Serial.print(packetHandler_->getRxPacketError(dxl_error));
  }
}

void setup(){
  Serial.begin(115200);
  while(!Serial);

  Serial.println("Start..");
  delay(1000);


  myOLLO.begin(1);//DMS Module must be connected at port 1.
  myOLLO.begin(2);//DMS Module must be connected at port 2.
  myOLLO.begin(3);//DMS Module must be connected at port 3.

  state = WAITING_FOR_ENTER;

  ///////////////////////////////
  /// Dynamixel
  ///////////////////////////////
  uint32_t baudrate_ = BAUDRATE;
  float  protocol_version_ = PROTOCOL_VERSION;

  Serial.print(fnInitDXL());

  fnWriteDXL(DXL_OPENED);
}

void loop(){
//   int sensor1_distance = myOLLO.read(1);
//   int sensor2_distance = myOLLO.read(2);
//   int sensor3_distance = myOLLO.read(3);
//
//   if (sensor1_distance > 650 && state == WAITING_FOR_ENTER)
//   {
//     state = ENTERED;
//   }
//   else if (sensor2_distance > 650 && state == ENTERED)
//   {
//     state = MUST_STOP;
//   }
//   else if (sensor3_distance > 650 && state == MUST_STOP)
//   {
//     state = IS_STOPPED;
//   }
//
//   Serial.print("DMS Sensor 1 ADC Value = ");
//   Serial.println(sensor1_distance); //read ADC value from OLLO port 1
//
//   Serial.print("DMS Sensor 2 ADC Value = ");
//   Serial.println(sensor2_distance); //read ADC value from OLLO port 2
// //  delay(60);
//   Serial.print("DMS Sensor 3 ADC Value = ");
//   Serial.println(sensor3_distance); //read ADC value from OLLO port 3
// // //  delay(60);
//
//   Serial.print("State : ");
//   Serial.println(state);
//
//   fnLevelControl();
//
//   delay(120);
}

void fnLevelControl()
{
  if (state == ENTERED)
  {
    if (is_started[0] == false)
    {
      //start stopwatch
      fnSetStopWatch();
      is_started[0] = true;
    }
    else
    {
      is_able_to_pass_ = false;

      if (fnGetTimeSinceStart() > 0.0 && fnGetTimeSinceStart() <= 3000.0)
      {
        fnWriteDXL(DXL_CLOSED);
      }
      // else if (fnGetTimeSinceStart() > 3000.0 && fnGetTimeSinceStart() <= 8000.0)
      // {
      //   fnWriteDXL(DXL_CLOSED);
      //   // is_able_to_pass_ = false;
      //   // is_started[1] = false;
      // }
      // else if (fnGetTimeSinceStart() > 8000.0)
      // {
      //   fnWriteDXL(LED_GREEN);
      //   // is_able_to_pass_ = true;
      //   // is_started[1] = false;
      // }
    }
    // digitalWrite(3, HIGH);
    // digitalWrite(5, LOW);
    // digitalWrite(6, LOW);
  }
  else if (state == MUST_STOP)
  {
    if (is_able_to_pass_ == false)
    {
      fnWriteDXL(DXL_CLOSED);

      // if (is_started[1] == false)
      // {
      //   //start stopwatch
      //   fnSetStopWatch();
      //   is_started[1] = true;
      // }
      // else
      // {
      //   if (fnGetTimeSinceStart() > 5000.0)
      //   {
      //     fnWriteDXL(LED_GREEN);
      //     is_able_to_pass_ = true;
      //     // is_started[1] = false;
      //   }
      //   else
      //   {
      //     fnWriteDXL(DXL_CLOSED);
      //     is_able_to_pass_ = false;
      //     // is_started[1] = false;
      //   }
      // }
    }
    else
    {
      // if (fnGetTimeSinceStart() > 50000.0)
      // {
        fnWriteDXL(DXL_OPENED);
        // is_able_to_pass_ = true;
        // is_started[1] = false;
      // }
    }

    // digitalWrite(3, LOW);
    // digitalWrite(5, HIGH);
    // digitalWrite(6, LOW);

  }
  // else if (state == IS_STOPPED)
  // {
  //   if (is_able_to_pass_ == false)
  //   {
  //     fnWriteDXL(ALL);
  //     // fnSetStopWatch();
  //     // is_started[1] = true;
  //   }
  //   else
  //   {
  //     // if (fnGetTimeSinceStart() > 50000.0)
  //     // {
  //       fnWriteDXL(LED_GREEN);
  //       // is_able_to_pass_ = true;
  //       // is_started[1] = false;
  //     // }
  //   }
  //
  //   // is_able_to_pass_ = false;
  //   // digitalWrite(3, LOW);
  //   // digitalWrite(5, LOW);
  //   // digitalWrite(6, HIGH);
  //   // fnWriteDXL(LED_GREEN);
  // }

  Serial.print(fnGetTimeSinceStart());
  Serial.println(" ");
}
//
// void fnWriteDXL(Level level_status)
// {
//   if (level_status == DXL_CLOSED)
//   {
//     digitalWrite(3, HIGH);
//     digitalWrite(5, LOW);
//     digitalWrite(6, LOW);
//   }
//   else if (level_status == DXL_OPENED)
//   {
//     digitalWrite(3, LOW);
//     digitalWrite(5, HIGH);
//     digitalWrite(6, LOW);
//   }
//   else if (level_status == LED_GREEN)
//   {
//     digitalWrite(3, LOW);
//     digitalWrite(5, LOW);
//     digitalWrite(6, HIGH);
//   }
//   else
//   {
//     digitalWrite(3, HIGH);
//     digitalWrite(5, HIGH);
//     digitalWrite(6, HIGH);
//   }
// }

double fnGetCurrentTime()
{
	return (double)millis();
}

double fnGetTimeSinceStart()
{
  double elapsed_time;

  elapsed_time = fnGetCurrentTime() - stopwatch_start_time_;
  if (elapsed_time < 0.0)
    stopwatch_start_time_ = fnGetCurrentTime();

  return elapsed_time;
}

void fnSetStopWatch()
{
  stopwatch_start_time_ = fnGetCurrentTime();
}
//
//
// #include <OLLO.h>
// OLLO myOLLO;
//
// // #define
//
// enum State { WAITING_FOR_ENTER, ENTERED, MUST_STOP, IS_STOPPED } state;
// enum Level { DXL_CLOSED, DXL_OPENED, LED_GREEN, ALL };
//
// double stopwatch_start_time_ = 0.0;
// bool is_started[3] = { false, false, false };
// bool is_able_to_pass_ = true;
//
// void setup(){
//   myOLLO.begin(1);//DMS Module must be connected at port 1.
//   myOLLO.begin(2);//DMS Module must be connected at port 2.
//   myOLLO.begin(3);//DMS Module must be connected at port 3.
//
//   pinMode(3, OUTPUT);
//   pinMode(5, OUTPUT);
//   pinMode(6, OUTPUT);
//
//   state = WAITING_FOR_ENTER;
// }
//
// void loop(){
//   int sensor1_distance = myOLLO.read(1);
//   int sensor2_distance = myOLLO.read(2);
//   int sensor3_distance = myOLLO.read(3);
//
//   if (sensor1_distance > 650 && state == WAITING_FOR_ENTER)
//   {
//     state = ENTERED;
//   }
//   else if (sensor2_distance > 650 && state == ENTERED)
//   {
//     state = MUST_STOP;
//   }
//   else if (sensor3_distance > 650 && state == MUST_STOP)
//   {
//     state = IS_STOPPED;
//   }
//
//   Serial.print("DMS Sensor 1 ADC Value = ");
//   Serial.println(sensor1_distance); //read ADC value from OLLO port 1
//
//   Serial.print("DMS Sensor 2 ADC Value = ");
//   Serial.println(sensor2_distance); //read ADC value from OLLO port 2
// //  delay(60);
//   Serial.print("DMS Sensor 3 ADC Value = ");
//   Serial.println(sensor3_distance); //read ADC value from OLLO port 3
// //  delay(60);
//
//   Serial.print("State : ");
//   Serial.println(state);
//
//   fnLevelControl();
//
//   delay(120);
// }
//
// void fnLevelControl()
// {
//   if (state == ENTERED)
//   {
//     if (is_started[0] == false)
//     {
//       //start stopwatch
//       fnSetStopWatch();
//       is_started[0] = true;
//     }
//     else
//     {
//       if (fnGetTimeSinceStart() > 0.0 && fnGetTimeSinceStart() <= 3000.0)
//       {
//         fnWriteDXL(DXL_OPENED);
//       }
//       else if (fnGetTimeSinceStart() > 3000.0 && fnGetTimeSinceStart() <= 8000.0)
//       {
//         fnWriteDXL(DXL_CLOSED);
//         is_able_to_pass_ = false;
//         // is_started[1] = false;
//       }
//       else if (fnGetTimeSinceStart() > 8000.0)
//       {
//         fnWriteDXL(LED_GREEN);
//         is_able_to_pass_ = true;
//         // is_started[1] = false;
//       }
//     }
//     // digitalWrite(3, HIGH);
//     // digitalWrite(5, LOW);
//     // digitalWrite(6, LOW);
//   }
//   else if (state == MUST_STOP)
//   {
//     if (is_able_to_pass_ == false)
//     {
//       if (is_started[1] == false)
//       {
//         //start stopwatch
//         fnSetStopWatch();
//         is_started[1] = true;
//       }
//       else
//       {
//         if (fnGetTimeSinceStart() > 5000.0)
//         {
//           fnWriteDXL(LED_GREEN);
//           is_able_to_pass_ = true;
//           // is_started[1] = false;
//         }
//         else
//         {
//           fnWriteDXL(DXL_CLOSED);
//           is_able_to_pass_ = false;
//           // is_started[1] = false;
//         }
//       }
//     }
//     // digitalWrite(3, LOW);
//     // digitalWrite(5, HIGH);
//     // digitalWrite(6, LOW);
//
//   }
//   else if (state == IS_STOPPED)
//   {
//     if (is_able_to_pass_ == false)
//     {
//       fnWriteDXL(ALL);
//       // fnSetStopWatch();
//       // is_started[1] = true;
//     }
//     else
//     {
//       // if (fnGetTimeSinceStart() > 50000.0)
//       // {
//         fnWriteDXL(LED_GREEN);
//         // is_able_to_pass_ = true;
//         // is_started[1] = false;
//       // }
//     }
//
//     // is_able_to_pass_ = false;
//     // digitalWrite(3, LOW);
//     // digitalWrite(5, LOW);
//     // digitalWrite(6, HIGH);
//     // fnWriteDXL(LED_GREEN);
//   }
//
//   Serial.print(fnGetTimeSinceStart());
//   Serial.println(" ");
// }
//
// void fnWriteDXL(Level level_status)
// {
//   if (level_status == DXL_CLOSED)
//   {
//     digitalWrite(3, HIGH);
//     digitalWrite(5, LOW);
//     digitalWrite(6, LOW);
//   }
//   else if (level_status == DXL_OPENED)
//   {
//     digitalWrite(3, LOW);
//     digitalWrite(5, HIGH);
//     digitalWrite(6, LOW);
//   }
//   else if (level_status == LED_GREEN)
//   {
//     digitalWrite(3, LOW);
//     digitalWrite(5, LOW);
//     digitalWrite(6, HIGH);
//   }
//   else
//   {
//     digitalWrite(3, HIGH);
//     digitalWrite(5, HIGH);
//     digitalWrite(6, HIGH);
//   }
// }
//
// double fnGetCurrentTime()
// {
// 	return (double)millis();
// }
//
// double fnGetTimeSinceStart()
// {
//   double elapsed_time;
//
//   elapsed_time = fnGetCurrentTime() - stopwatch_start_time_;
//   if (elapsed_time < 0.0)
//     stopwatch_start_time_ = fnGetCurrentTime();
//
//   return elapsed_time;
// }
//
// void fnSetStopWatch()
// {
//   stopwatch_start_time_ = fnGetCurrentTime();
// }
