/* RBiz Challenge TurtleBot3 Auto Race - Traffic Light

|-----|
||---||
|| R ||
||---||
|     |
||---||
|| Y ||
||---||
|     |
||---||
|| G ||
||---||            |---------|            |---------|            |---------|
|-----|            | O-----0 |            | O-----0 |            | O-----0 |
  | |              |   DMS   |            |   DMS   |            |   DMS   |
  | |              | Sensor1 |            | Sensor2 |            | Sensor3 |
  | |              |         |            |         |            |         |
=======------------===========------------===========------------===========

 created 1 October 2017
 by ROBOTIS CO,.LTD.

 author : Leon Ryuwoon Jung
 */

#include "traffic_light.h"

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<rbiz_autorace_msgs::DoIt> subInitStateTrafficLight("init_state/traffic_light",cbInitStateTrafficLight);
ros::Subscriber<rbiz_autorace_msgs::DoIt> subTestStateTrafficLight("test_state/traffic_light",cbTestStateTrafficLight);

/*******************************************************************************
* Publisher
*******************************************************************************/
// DMS, battery, etc.
rbiz_autorace_msgs::SensorStateTrafficLight msgSensorStateTrafficLight;
ros::Publisher pubSensorStateTrafficLight("sensor_state/traffic_light", &msgSensorStateTrafficLight);

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.subscribe(subInitStateTrafficLight);
  nh.subscribe(subTestStateTrafficLight);
  nh.advertise(pubSensorStateTrafficLight);

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
  fnInitStateTrafficLight();
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{  
  fnGetButtonPressed();

  fnReceiveSensorDistance();

  fnCheckVehicleStatus();

  fnLEDControl();

  fnControlLED();

  pbSensorState();

  nh.spinOnce();
}

/*******************************************************************************
* Publish msgs (Sensor data: Power, etc.)
*******************************************************************************/
void pbSensorState()
{
  msgSensorStateTrafficLight.stamp = nh.now();
  msgSensorStateTrafficLight.elapsed_time = fnGetTimeSinceStart();
  msgSensorStateTrafficLight.sensor_distance[0] = sensor_distance[0];
  msgSensorStateTrafficLight.sensor_distance[1] = sensor_distance[1];
  msgSensorStateTrafficLight.sensor_distance[2] = sensor_distance[2];
  msgSensorStateTrafficLight.is_started[0] = is_started[0];
  msgSensorStateTrafficLight.is_started[1] = is_started[1];
  msgSensorStateTrafficLight.is_started[2] = is_started[2];
  msgSensorStateTrafficLight.is_able_to_pass = is_able_to_pass_;
  msgSensorStateTrafficLight.vehicle_state = vehicle_state_;
  msgSensorStateTrafficLight.led_color = led_color_;
  msgSensorStateTrafficLight.battery = fncheckVoltage();

  pubSensorStateTrafficLight.publish(&msgSensorStateTrafficLight);
}

/*******************************************************************************
* Callback function
*******************************************************************************/

void cbInitStateTrafficLight(const rbiz_autorace_msgs::DoIt& msgInitStateTrafficLight)
{
  if (msgInitStateTrafficLight.doIt == true)
  {
    fnInitStateTrafficLight();
  }
}

void cbTestStateTrafficLight(const rbiz_autorace_msgs::DoIt& msgTestStateTrafficLight)
{
  if (msgTestStateTrafficLight.doIt == true)
  {
    fnTestStateTrafficLight();
  }
}

/*******************************************************************************
* Normal function
*******************************************************************************/

void fnGetButtonPressed()
{
  if (ollo.read(4, TOUCH_SENSOR))
  {
    fnInitStateTrafficLight();
  }
}

void fnInitStateTrafficLight()
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
  led_color_ = LED_GREEN;
  mode_ = ACTIVE_MODE;
    // mode_ = TEST_MODE;
}

void fnTestStateTrafficLight()
{
  fnInitStateTrafficLight();
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

void fnLEDControl()
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
        if (fnGetTimeSinceStart() > 0.0 && fnGetTimeSinceStart() <= 3000.0)
        {
          led_color_ = LED_YELLOW;
        }
        else if (fnGetTimeSinceStart() > 3000.0)
        {
          led_color_ = LED_RED;
          is_able_to_pass_ = false;
        }
      }
    }
    else if (vehicle_state_ == MUST_STOP)
    {
      if (is_started[1] == false)
      {
        if (fnGetTimeSinceStart() <= 3000.0)
        {
          led_color_ = LED_YELLOW;
        }
        else
        {
          led_color_ = LED_RED;
          is_able_to_pass_ = false;

          fnSetStopWatch();
          is_started[1] = true;
        }
      }
      else
      {
        if (fnGetTimeSinceStart() <= 5000.0)
        {
          led_color_ = LED_RED;
          is_able_to_pass_ = false;
        }
        else
        {
          led_color_ = LED_GREEN;
          is_able_to_pass_ = true;
        }
      }
    }
    else if (vehicle_state_ == PASSED)
    {
      if (is_started[2] == false)
      {
        //start stopwatch
        fnSetStopWatch();
        is_started[2] = true;
      }
      else
      {
        if (led_turn_ == 1)
        {
          if (is_able_to_pass_ == false)
          {
            led_color_ = LED_YELLOW;
          }
          else
          {
            led_color_ = LED_GREEN;
          }
        }
        else
        {
          led_color_ = LED_ALL_LOW;
        }

        led_turn_ = 1 - led_turn_;

        delay(200);
      }
    }
  }
  else if (mode_ == TEST_MODE)
  {
    if (vehicle_state_ == ENTERED)
    {
      led_color_ = LED_RED;
    }
    else if (vehicle_state_ == MUST_STOP)
    {
      led_color_ = LED_YELLOW;
    }
    else if (vehicle_state_ == PASSED)
    {
      led_color_ = LED_GREEN;

      fnInitStateTrafficLight();
    }
  }
}

void fnControlLED()
{
  if (led_color_ == LED_RED)
  {
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
  }
  else if (led_color_ == LED_YELLOW)
  {
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
    digitalWrite(11, LOW);
  }
  else if (led_color_ == LED_GREEN)
  {
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, HIGH);
  }
  else
  {
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
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
//
//
// // #include <OLLO.h>
// // OLLO myOLLO;
// //
// // // #define
// //
// // enum State { WAITING_FOR_ENTER, ENTERED, MUST_STOP, IS_STOPPED } state;
// // enum Color { LED_RED, LED_YELLOW, LED_GREEN, ALL };
// //
// // double stopwatch_start_time_ = 0.0;
// // bool is_started[3] = { false, false, false };
// // bool is_able_to_pass_ = true;
// //
// // void setup(){
// //   myOLLO.begin(1);//DMS Module must be connected at port 1.
// //   myOLLO.begin(2);//DMS Module must be connected at port 2.
// //   myOLLO.begin(3);//DMS Module must be connected at port 3.
// //
// //   pinMode(9, OUTPUT);
// //   pinMode(10, OUTPUT);
// //   pinMode(11, OUTPUT);
// //
// //   state = WAITING_FOR_ENTER;
// // }
// //
// // void loop(){
// //   int sensor1_distance = myOLLO.read(1);
// //   int sensor2_distance = myOLLO.read(2);
// //   int sensor3_distance = myOLLO.read(3);
// //
// //   if (sensor1_distance > 650 && state == WAITING_FOR_ENTER)
// //   {
// //     state = ENTERED;
// //   }
// //   else if (sensor2_distance > 650 && state == ENTERED)
// //   {
// //     state = MUST_STOP;
// //   }
// //   else if (sensor3_distance > 650 && state == MUST_STOP)
// //   {
// //     state = IS_STOPPED;
// //   }
// //
// //   Serial.print("DMS Sensor 1 ADC Value = ");
// //   Serial.println(sensor1_distance); //read ADC value from OLLO port 1
// //
// //   Serial.print("DMS Sensor 2 ADC Value = ");
// //   Serial.println(sensor2_distance); //read ADC value from OLLO port 2
// // //  delay(60);
// //   Serial.print("DMS Sensor 3 ADC Value = ");
// //   Serial.println(sensor3_distance); //read ADC value from OLLO port 3
// // //  delay(60);
// //
// //   Serial.print("State : ");
// //   Serial.println(state);
// //
// //   fnLEDControl();
// //
// //   delay(120);
// // }
// //
// // void fnLEDControl()
// // {
// //   if (state == ENTERED)
// //   {
// //     if (is_started[0] == false)
// //     {
// //       //start stopwatch
// //       fnSetStopWatch();
// //       is_started[0] = true;
// //     }
// //     else
// //     {
// //       if (fnGetTimeSinceStart() > 0.0 && fnGetTimeSinceStart() <= 3000.0)
// //       {
// //         fnControlLED(LED_YELLOW);
// //       }
// //       else if (fnGetTimeSinceStart() > 3000.0 && fnGetTimeSinceStart() <= 8000.0)
// //       {
// //         fnControlLED(LED_RED);
// //         is_able_to_pass_ = false;
// //         // is_started[1] = false;
// //       }
// //       // else if (fnGetTimeSinceStart() > 8000.0)
// //       // {
// //       //   fnControlLED(LED_GREEN);
// //       //   is_able_to_pass_ = true;
// //       //   // is_started[1] = false;
// //       // }
// //     }
// //     // digitalWrite(3, HIGH);
// //     // digitalWrite(5, LOW);
// //     // digitalWrite(6, LOW);
// //   }
// //   else if (state == MUST_STOP)
// //   {
// //     // if (is_able_to_pass_ == false)
// //     // {
// //       if (is_started[1] == false)
// //       {
// //         //start stopwatch
// //         fnSetStopWatch();
// //         is_started[1] = true;
// //       }
// //       else
// //       {
// //         if (fnGetTimeSinceStart() > 5000.0)
// //         {
// //           fnControlLED(LED_GREEN);
// //           is_able_to_pass_ = true;
// //           // is_started[1] = false;
// //         }
// //         else
// //         {
// //           fnControlLED(LED_RED);
// //           is_able_to_pass_ = false;
// //           // is_started[1] = false;
// //         }
// //       }
// //     // }
// //     // digitalWrite(3, LOW);
// //     // digitalWrite(5, HIGH);
// //     // digitalWrite(6, LOW);
// //
// //   }
// //   else if (state == IS_STOPPED)
// //   {
// //     if (is_able_to_pass_ == false)
// //     {
// //       fnControlLED(ALL);
// //       // fnSetStopWatch();
// //       // is_started[1] = true;
// //     }
// //     else
// //     {
// //       // if (fnGetTimeSinceStart() > 50000.0)
// //       // {
// //         fnControlLED(LED_GREEN);
// //         // is_able_to_pass_ = true;
// //         // is_started[1] = false;
// //       // }
// //     }
// //
// //     // is_able_to_pass_ = false;
// //     // digitalWrite(3, LOW);
// //     // digitalWrite(5, LOW);
// //     // digitalWrite(6, HIGH);
// //     // fnControlLED(LED_GREEN);
// //   }
// //
// //   Serial.print(fnGetTimeSinceStart());
// //   Serial.println(" ");
// // }
// //
// // void fnControlLED(Color led_color)
// // {
// //   if (led_color == LED_RED)
// //   {
// //     digitalWrite(9, HIGH);
// //     digitalWrite(10, LOW);
// //     digitalWrite(11, LOW);
// //   }
// //   else if (led_color == LED_YELLOW)
// //   {
// //     digitalWrite(9, LOW);
// //     digitalWrite(10, HIGH);
// //     digitalWrite(11, LOW);
// //   }
// //   else if (led_color == LED_GREEN)
// //   {
// //     digitalWrite(9, LOW);
// //     digitalWrite(10, LOW);
// //     digitalWrite(11, HIGH);
// //   }
// //   else
// //   {
// //     digitalWrite(9, HIGH);
// //     digitalWrite(10, HIGH);
// //     digitalWrite(11, HIGH);
// //   }
// // }
// //
// // double fnGetCurrentTime()
// // {
// // 	return (double)millis();
// // }
// //
// // double fnGetTimeSinceStart()
// // {
// //   double elapsed_time;
// //
// //   elapsed_time = fnGetCurrentTime() - stopwatch_start_time_;
// //   if (elapsed_time < 0.0)
// //     stopwatch_start_time_ = fnGetCurrentTime();
// //
// //   return elapsed_time;
// // }
// //
// // void fnSetStopWatch()
// // {
// //   stopwatch_start_time_ = fnGetCurrentTime();
// // }
