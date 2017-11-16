/* RBiz Challenge TurtleBot3 Auto Race - Level Crossing


|-----|
||---||----|-------|-------|-------|-------|
||DXL||    |   W   |   R   |   W   |   R   |
||---||----|-------|-------|-------|-------|
|     |
|     |
|     |
|     |  |---------|            |---------|            |---------|
|-----|  | O-----0 |            | O-----0 |            | O-----0 |
  | |    |   DMS   |            |   DMS   |            |   DMS   |
  | |    | Sensor1 |            | Sensor2 |            | Sensor3 |
  | |    |         |            |         |            |         |
=======------------===========------------===========------------===========

created 1 October 2017
by ROBOTIS CO,.LTD.

author : Leon Ryuwoon Jung
*/

#include "level_crossing.h"

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<rbiz_autorace_msgs::DoIt> subInitStateLevelCrossing("init_state/level_crossing",cbInitStateLevelCrossing);
ros::Subscriber<rbiz_autorace_msgs::DoIt> subTestStateLevelCrossing("test_state/level_crossing",cbTestStateLevelCrossing);

/*******************************************************************************
* Publisher
*******************************************************************************/
// DMS, battery, etc.
rbiz_autorace_msgs::SensorStateLevelCrossing msgSensorStateLevelCrossing;
ros::Publisher pubSensorStateLevelCrossing("sensor_state/level_crossing", &msgSensorStateLevelCrossing);

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.subscribe(subInitStateLevelCrossing);
  nh.subscribe(subTestStateLevelCrossing);
  nh.advertise(pubSensorStateLevelCrossing);

  nh.loginfo("Connected to OpenCR board!");

  // Settings for OLLO DMS
  ollo.begin(1);  //DMS Module must be connected at port 1.
  ollo.begin(2);  //DMS Module must be connected at port 2.
  ollo.begin(3);  //DMS Module must be connected at port 3.

  // Settings for OLLO TS-10
  ollo.begin(4, TOUCH_SENSOR);

  // Setting for Dynamixel motors
  motorDriver.init();

  // Init Paramters
  fnInitStateLevelCrossing();
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  fnGetButtonPressed();

  fnReceiveSensorDistance();

  fnCheckVehicleStatus();

  fnLevelControl();

  fnControlLevelPose();

  pbSensorState();

  nh.spinOnce();
}

/*******************************************************************************
* Publish msgs (Sensor data: Power, etc.)
*******************************************************************************/
void pbSensorState()
{
  msgSensorStateLevelCrossing.stamp = nh.now();
  msgSensorStateLevelCrossing.elapsed_time = fnGetTimeSinceStart();
  msgSensorStateLevelCrossing.sensor_distance[0] = sensor_distance[0];
  msgSensorStateLevelCrossing.sensor_distance[1] = sensor_distance[1];
  msgSensorStateLevelCrossing.sensor_distance[2] = sensor_distance[2];
  msgSensorStateLevelCrossing.is_started[0] = is_started[0];
  msgSensorStateLevelCrossing.is_started[1] = is_started[1];
  msgSensorStateLevelCrossing.is_started[2] = is_started[2];
  msgSensorStateLevelCrossing.is_able_to_pass = is_able_to_pass_;
  msgSensorStateLevelCrossing.vehicle_state = vehicle_state_;
  msgSensorStateLevelCrossing.level_status = level_status_;
  msgSensorStateLevelCrossing.battery = fncheckVoltage();

  pubSensorStateLevelCrossing.publish(&msgSensorStateLevelCrossing);
}

/*******************************************************************************
* Callback function
*******************************************************************************/

void cbInitStateLevelCrossing(const rbiz_autorace_msgs::DoIt& msgInitStateLevelCrossing)
{
  if (msgInitStateLevelCrossing.doIt == true)
  {
    fnInitStateLevelCrossing();
  }
}

void cbTestStateLevelCrossing(const rbiz_autorace_msgs::DoIt& msgTestStateLevelCrossing)
{
  if (msgTestStateLevelCrossing.doIt == true)
  {
    fnTestStateLevelCrossing();
  }
}

/*******************************************************************************
* Normal function
*******************************************************************************/

void fnGetButtonPressed()
{
  if (ollo.read(4, TOUCH_SENSOR))
  {
    fnInitStateLevelCrossing();
  }
}

void fnInitStateLevelCrossing()
{
  fnSetStopWatch();

  sensor_distance[0] = 0;
  sensor_distance[1] = 0;
  sensor_distance[2] = 0;

  is_started[0] = false;
  is_started[1] = false;
  is_started[2] = false;

  is_able_to_pass_ = true;
  vehicle_state_ = WAITING_FOR_ENTER;
  level_status_ = LEVEL_OPENED;
  mode_ = ACTIVE_MODE;
    // mode_ = TEST_MODE;
}

void fnTestStateLevelCrossing()
{
  fnInitStateLevelCrossing();
  mode_ = TEST_MODE;
}

void fnReceiveSensorDistance()
{
  sensor_distance[0] = ollo.read(1);
  sensor_distance[1] = ollo.read(2);
  sensor_distance[2] = ollo.read(3);

  // Serial.print("DMS Sensor 1 ADC Value = ");
  // Serial.println(sensor_distance[0]); //read ADC value from OLLO port 1
  //
  // Serial.print("DMS Sensor 2 ADC Value = ");
  // Serial.println(sensor_distance[1]); //read ADC value from OLLO port 2
  //
  // Serial.print("DMS Sensor 3 ADC Value = ");
  // Serial.println(sensor_distance[2]); //read ADC value from OLLO port 3

  delay(100);
}

void fnCheckVehicleStatus()
{
  if (sensor_distance[0] > DISTANCE_THRESHOLD_PASS && vehicle_state_ == WAITING_FOR_ENTER)
  {
    vehicle_state_ = ENTERED;
  }
  else if (sensor_distance[1] > DISTANCE_THRESHOLD_PASS && vehicle_state_ == ENTERED)
  {
    vehicle_state_ = MUST_STOP;
  }
  else if (sensor_distance[2] > DISTANCE_THRESHOLD_PASS && vehicle_state_ == MUST_STOP)
  {
    vehicle_state_ = PASSED;
  }

  // Serial.print("State : ");
  // Serial.println(vehicle_state_);
}

void fnLevelControl()
{
  if (mode_ == ACTIVE_MODE)
  {
    if (vehicle_state_ == ENTERED)
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

        level_status_ = LEVEL_CLOSED;


        // }
        // else if (fnGetTimeSinceStart() > 3000.0 && fnGetTimeSinceStart() <= 8000.0)
        // {
        //   level_status_ = LEVEL_CLOSED;
        //   // is_able_to_pass_ = false;
        //   // is_started[1] = false;
        // }
        // else if (fnGetTimeSinceStart() > 8000.0)
        // {
        //   level_status_ = LED_GREEN);
        //  // is_able_to_pass_ = true;
        //   // is_started[1] = false;
        // }
      }
    }
    else if (vehicle_state_ == MUST_STOP)
    {
      // if (is_able_to_pass_ == false)
      // {
        // level_status_ = LEVEL_CLOSED;

        if (is_started[1] == false)
        {
          //start stopwatch
          fnSetStopWatch();
          is_started[1] = true;
        }
        else
        {
          if (fnGetTimeSinceStart() > 5000.0)
          {
            level_status_ = LEVEL_OPENED;


            is_able_to_pass_ = true;
            // is_started[1] = false;
          }
          else
          {
            level_status_ = LEVEL_CLOSED;


            is_able_to_pass_ = false;
            // is_started[1] = false;
          }
        }
      // }
      // else
      // {
      //   // if (fnGetTimeSinceStart() > 50000.0)
      //   // {
      //     level_status_ = LEVEL_OPENED;
      //     // is_able_to_pass_ = true;
      //     // is_started[1] = false;
      //   // }
      // }
    }
    else if (vehicle_state_ == PASSED)
    {
      if (is_started[2] == false)
      {
        fnSetStopWatch();
        is_started[2] = true;
      }
      else
      {
        if (is_able_to_pass_ == false)
        {
          level_status_ = LEVEL_CLOSED;


          // level_status_ = ALL);
          // fnSetStopWatch);
          // is_started[2] = true;
        }
        else
        {
          level_status_ = LEVEL_OPENED;


          // if (fnGetTimeSinceStart() > 50000.0)
          // {
            // level_status_ = LED_GREEN);
            //is_able_to_pass_ = true;
            // is_started[1] = false;
          // }
        }
      }

      // is_able_to_pass_ = false;
    }
  }
  else if (mode_ == TEST_MODE)
  {
    if (vehicle_state_ == ENTERED)
    {
      level_status_ = LEVEL_CLOSED;
    }
    else if (vehicle_state_ == MUST_STOP)
    {
      level_status_ = LEVEL_MIDDLE;
    }
    else if (vehicle_state_ == PASSED)
    {
      level_status_ = LEVEL_OPENED;

      fnInitStateLevelCrossing();
    }
  }

  // Serial.print(fnGetTimeSinceStart());
  // Serial.println(" ");
}

void fnControlLevelPose()
{
  if (level_status_ == LEVEL_OPENED)
  {
    motorDriver.controlPosition(DXL_ID, DXL_POSITION_VALUE_OPENED);
  }
  else if (level_status_ == LEVEL_CLOSED)
  {
    motorDriver.controlPosition(DXL_ID, DXL_POSITION_VALUE_CLOSED);
  }
  else if (level_status_ == LEVEL_MIDDLE)
  {
    motorDriver.controlPosition(DXL_ID, DXL_POSITION_VALUE_MIDDLE);
  }
}

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

/*******************************************************************************
* Check voltage
*******************************************************************************/
float fncheckVoltage()
{
  float vol_value;

  vol_value = getPowerInVoltage();

  return vol_value;
}
