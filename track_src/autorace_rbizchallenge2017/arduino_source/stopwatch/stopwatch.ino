/* RBiz Challenge TurtleBot3 Auto Race - Stopwatch












            |---------|            |---------|            |---------|
            | O-----0 |            | O-----0 |            | O-----0 |
            |   DMS   |            |   DMS   |            |   DMS   |
            | Sensor1 |            | Sensor2 |            | Sensor3 |
            |         |            |         |            |         |
------------===========------------===========------------===========

 created 1 October 2017
 by ROBOTIS CO,.LTD.

 author : Leon Ryuwoon Jung
 */

#include "stopwatch.h"

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<rbiz_autorace_msgs::DoIt> subInitStateStopwatch("init_state/stopwatch",cbInitStateStopwatch);
ros::Subscriber<rbiz_autorace_msgs::DoIt> subTestStateStopwatch("test_state/stopwatch",cbTestStateStopwatch);

/*******************************************************************************
* Publisher
*******************************************************************************/
// DMS, battery, etc.
rbiz_autorace_msgs::SensorStateStopwatch msgSensorStateStopwatch;
ros::Publisher pubSensorStateStopwatch("sensor_state/stopwatch", &msgSensorStateStopwatch);

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.subscribe(subInitStateStopwatch);
  nh.subscribe(subTestStateStopwatch);
  nh.advertise(pubSensorStateStopwatch);

  nh.loginfo("Connected to OpenCR board!");

  // Settings for OLLO DMS
  ollo.begin(1);  //DMS Module must be connected at port 1.
  ollo.begin(2);  //DMS Module must be connected at port 2.
  ollo.begin(3);  //DMS Module must be connected at port 3.

  // Settings for OLLO TS-10
  ollo.begin(4, TOUCH_SENSOR);

  // Pins for LED
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  // Init Paramters
  fnInitStateStopwatch();
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  fnGetButtonPressed();

  fnReceiveSensorDistance();

  fnCheckVehicleStatus();

  fnStopwatchControl();

  fnGetLapTime();

  pbSensorState();

  nh.spinOnce();
}

/*******************************************************************************
* Publish msgs (Sensor data: Power, etc.)
*******************************************************************************/
void pbSensorState()
{
  msgSensorStateStopwatch.stamp = nh.now();
  msgSensorStateStopwatch.elapsed_time = fnGetTimeSinceStart();
  msgSensorStateStopwatch.sensor_distance[0] = sensor_distance[0];
  msgSensorStateStopwatch.sensor_distance[1] = sensor_distance[1];
  msgSensorStateStopwatch.sensor_distance[2] = sensor_distance[2];
  msgSensorStateStopwatch.is_started[0] = is_started[0];
  msgSensorStateStopwatch.is_started[1] = is_started[1];
  msgSensorStateStopwatch.is_started[2] = is_started[2];
  msgSensorStateStopwatch.is_finished = is_finished;
  msgSensorStateStopwatch.lap_time = lap_time;
  msgSensorStateStopwatch.vehicle_state = vehicle_state_;
  // msgSensorStateStopwatch.led_color = led_color_;
  msgSensorStateStopwatch.battery = fncheckVoltage();

  pubSensorStateStopwatch.publish(&msgSensorStateStopwatch);
}

/*******************************************************************************
* Callback function
*******************************************************************************/

void cbInitStateStopwatch(const rbiz_autorace_msgs::DoIt& msgInitStateStopwatch)
{
  if (msgInitStateStopwatch.doIt == true)
  {
    fnInitStateStopwatch();
  }
}

void cbTestStateStopwatch(const rbiz_autorace_msgs::DoIt& msgTestStateStopwatch)
{
  if (msgTestStateStopwatch.doIt == true)
  {
    fnTestStateStopwatch();
  }
}

/*******************************************************************************
* Normal function
*******************************************************************************/

void fnGetButtonPressed()
{
  if (ollo.read(4, TOUCH_SENSOR))
  {
    fnInitStateStopwatch();
  }
}

void fnInitStateStopwatch()
{
  fnSetStopWatch();

  sensor_distance[0] = 0;
  sensor_distance[1] = 0;
  sensor_distance[2] = 0;

  is_started[0] = false;
  is_started[1] = false;
  is_started[2] = false;

  is_finished = false;
  vehicle_state_ = WAITING_FOR_START;
  // lap_time = fnGetLapTime();
  // led_color_ = LED_GREEN;
  mode_ = ACTIVE_MODE;
    // mode_ = TEST_MODE;
}

void fnTestStateStopwatch()
{
  fnInitStateStopwatch();
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
  if (sensor_distance[1] > DISTANCE_THRESHOLD_PASS && vehicle_state_ == WAITING_FOR_START)
  {
    vehicle_state_ = STARTED;
  }
  else if (sensor_distance[2] > DISTANCE_THRESHOLD_PASS && vehicle_state_ == STARTED)
  {
    vehicle_state_ = IS_GOING_TOWARD_RIGHT_DIRECTION;
  }
  else if (sensor_distance[0] > DISTANCE_THRESHOLD_PASS && vehicle_state_ == IS_GOING_TOWARD_RIGHT_DIRECTION)
  {
    vehicle_state_ = IS_ABOUT_TO_FINISH;
  }
  else if (sensor_distance[1] > DISTANCE_THRESHOLD_PASS && vehicle_state_ == IS_ABOUT_TO_FINISH)
  {
    vehicle_state_ = FINISHED;
  }

  // Serial.print("State : ");
  // Serial.println(vehicle_state_);
}

void fnStopwatchControl()
{
  if (mode_ == ACTIVE_MODE)
  {
    if (vehicle_state_ == STARTED)
    {
      if (is_started[0] == false)
      {
        //start stopwatch
        fnSetStopWatch();
        is_started[0] = true;
      }
      else
      {
        // if (fnGetTimeSinceStart() > 0.0 && fnGetTimeSinceStart() <= 3000.0)
        // {
        //   led_color_ = LED_YELLOW;
        // }
        // else if (fnGetTimeSinceStart() > 3000.0)
        // {
        //   led_color_ = LED_RED;
        //   is_able_to_pass_ = false;
        // }
      }
    }
    else if (vehicle_state_ == IS_GOING_TOWARD_RIGHT_DIRECTION)
    {
      // if (is_started[1] == false)
      // {
      //   if (fnGetTimeSinceStart() <= 3000.0)
      //   {
      //     led_color_ = LED_YELLOW;
      //   }
      //   else
      //   {
      //     led_color_ = LED_RED;
      //     is_able_to_pass_ = false;
      //
      //     fnSetStopWatch();
      //     is_started[1] = true;
      //   }
      // }
      // else
      // {
      //   if (fnGetTimeSinceStart() <= 5000.0)
      //   {
      //     led_color_ = LED_RED;
      //     is_able_to_pass_ = false;
      //   }
      //   else
      //   {
      //     led_color_ = LED_GREEN;
      //     is_able_to_pass_ = true;
      //   }
      // }
    }
    else if (vehicle_state_ == IS_ABOUT_TO_FINISH)
    {
    //   if (is_started[2] == false)
    //   {
    //     //start stopwatch
    //     fnSetStopWatch();
    //     is_started[2] = true;
    //   }
    //   else
    //   {
    //     if (led_turn_ == 1)
    //     {
    //       if (is_able_to_pass_ == false)
    //       {
    //         led_color_ = LED_YELLOW;
    //       }
    //       else
    //       {
    //         led_color_ = LED_GREEN;
    //       }
    //     }
    //     else
    //     {
    //       led_color_ = LED_ALL_LOW;
    //     }
    //
    //     led_turn_ = 1 - led_turn_;
    //
    //     delay(200);
    //   }
    // }
    }
    else if (vehicle_state_ == FINISHED)
    {
      is_finished = true;
    //   if (is_started[2] == false)
    //   {
    //     //start stopwatch
    //     fnSetStopWatch();
    //     is_started[2] = true;
    //   }
    //   else
    //   {
    //     if (led_turn_ == 1)
    //     {
    //       if (is_able_to_pass_ == false)
    //       {
    //         led_color_ = LED_YELLOW;
    //       }
    //       else
    //       {
    //         led_color_ = LED_GREEN;
    //       }
    //     }
    //     else
    //     {
    //       led_color_ = LED_ALL_LOW;
    //     }
    //
    //     led_turn_ = 1 - led_turn_;
    //
    //     delay(200);
    //   }
    // }
    }
    // else if (mode_ == FINISHED)
    // {
    //
    // // if (vehicle_state_ == ENTERED)
    // // {
    // //   led_color_ = LED_RED;
    // // }
    // // else if (vehicle_state_ == MUST_STOP)
    // // {
    // //   led_color_ = LED_YELLOW;
    // // }
    // // else if (vehicle_state_ == PASSED)
    // // {
    // //   led_color_ = LED_GREEN;
    // //
    // //   fnInitStateStopwatch();
    // // }
    // }
  }
}

void fnGetLapTime()
{
  if (is_finished == true)
  {

  }
  else if (!is_started[0])
  {
    lap_time = 0.0;
  }
  else
  {
    lap_time = fnGetTimeSinceStart();
  }
}

// void fnControlLED()
// {
//   if (led_color_ == LED_RED)
//   {
//     digitalWrite(9, HIGH);
//     digitalWrite(10, LOW);
//     digitalWrite(11, LOW);
//   }
//   else if (led_color_ == LED_YELLOW)
//   {
//     digitalWrite(9, LOW);
//     digitalWrite(10, HIGH);
//     digitalWrite(11, LOW);
//   }
//   else if (led_color_ == LED_GREEN)
//   {
//     digitalWrite(9, LOW);
//     digitalWrite(10, LOW);
//     digitalWrite(11, HIGH);
//   }
//   else
//   {
//     digitalWrite(9, LOW);
//     digitalWrite(10, LOW);
//     digitalWrite(11, LOW);
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

/*******************************************************************************
* Check voltage
*******************************************************************************/
float fncheckVoltage()
{
  float vol_value;

  vol_value = getPowerInVoltage();

  return vol_value;
}
