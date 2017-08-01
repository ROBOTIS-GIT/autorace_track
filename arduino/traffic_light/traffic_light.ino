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
// for two DMSs
#include <OLLO.h>
OLLO myOLLO;

// #define

enum State { WAITING_FOR_ENTER, ENTERED, MUST_STOP, IS_STOPPED } state;
enum Color { LED_RED, LED_YELLOW, LED_GREEN, ALL };

double stopwatch_start_time_ = 0.0;
bool is_started[3] = { false, false, false };
bool is_able_to_pass_ = true;

void setup(){
  myOLLO.begin(1);//DMS Module must be connected at port 1.
  myOLLO.begin(2);//DMS Module must be connected at port 2.
  // myOLLO.begin(3);//DMS Module must be connected at port 3.

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  state = WAITING_FOR_ENTER;
}

void loop(){
  int sensor1_distance = myOLLO.read(1);
  int sensor2_distance = myOLLO.read(2);
  // int sensor3_distance = myOLLO.read(3);

  if (sensor1_distance > 650 && state == WAITING_FOR_ENTER)
  {
    state = ENTERED;
  }
  else if (sensor2_distance > 650 && state == ENTERED)
  {
    state = MUST_STOP;
  }
  // else if (sensor3_distance > 650 && state == MUST_STOP)
  // {
  //   state = IS_STOPPED;
  // }

  Serial.print("DMS Sensor 1 ADC Value = ");
  Serial.println(sensor1_distance); //read ADC value from OLLO port 1

  Serial.print("DMS Sensor 2 ADC Value = ");
  Serial.println(sensor2_distance); //read ADC value from OLLO port 2
//  delay(60);
//   Serial.print("DMS Sensor 3 ADC Value = ");
//   Serial.println(sensor3_distance); //read ADC value from OLLO port 3
// //  delay(60);

  Serial.print("State : ");
  Serial.println(state);

  fnLEDControl();

  delay(120);
}

void fnLEDControl()
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
        fnWriteLED(LED_YELLOW);
      }
      else if (fnGetTimeSinceStart() > 3000.0 && fnGetTimeSinceStart() <= 8000.0)
      {
        fnWriteLED(LED_RED);
        // is_able_to_pass_ = false;
        // is_started[1] = false;
      }
      else if (fnGetTimeSinceStart() > 8000.0)
      {
        fnWriteLED(LED_GREEN);
        // is_able_to_pass_ = true;
        // is_started[1] = false;
      }
    }
    // digitalWrite(3, HIGH);
    // digitalWrite(5, LOW);
    // digitalWrite(6, LOW);
  }
  else if (state == MUST_STOP)
  {
    if (is_able_to_pass_ == false)
    {
      fnWriteLED(ALL);

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
      //     fnWriteLED(LED_GREEN);
      //     is_able_to_pass_ = true;
      //     // is_started[1] = false;
      //   }
      //   else
      //   {
      //     fnWriteLED(LED_RED);
      //     is_able_to_pass_ = false;
      //     // is_started[1] = false;
      //   }
      // }
    }
    else
    {
      // if (fnGetTimeSinceStart() > 50000.0)
      // {
        fnWriteLED(LED_GREEN);
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
  //     fnWriteLED(ALL);
  //     // fnSetStopWatch();
  //     // is_started[1] = true;
  //   }
  //   else
  //   {
  //     // if (fnGetTimeSinceStart() > 50000.0)
  //     // {
  //       fnWriteLED(LED_GREEN);
  //       // is_able_to_pass_ = true;
  //       // is_started[1] = false;
  //     // }
  //   }
  //
  //   // is_able_to_pass_ = false;
  //   // digitalWrite(3, LOW);
  //   // digitalWrite(5, LOW);
  //   // digitalWrite(6, HIGH);
  //   // fnWriteLED(LED_GREEN);
  // }

  Serial.print(fnGetTimeSinceStart());
  Serial.println(" ");
}

void fnWriteLED(Color led_color)
{
  if (led_color == LED_RED)
  {
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
  }
  else if (led_color == LED_YELLOW)
  {
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
    digitalWrite(11, LOW);
  }
  else if (led_color == LED_GREEN)
  {
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, HIGH);
  }
  else
  {
    digitalWrite(9, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(11, HIGH);
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


// #include <OLLO.h>
// OLLO myOLLO;
//
// // #define
//
// enum State { WAITING_FOR_ENTER, ENTERED, MUST_STOP, IS_STOPPED } state;
// enum Color { LED_RED, LED_YELLOW, LED_GREEN, ALL };
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
//   pinMode(9, OUTPUT);
//   pinMode(10, OUTPUT);
//   pinMode(11, OUTPUT);
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
//   fnLEDControl();
//
//   delay(120);
// }
//
// void fnLEDControl()
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
//         fnWriteLED(LED_YELLOW);
//       }
//       else if (fnGetTimeSinceStart() > 3000.0 && fnGetTimeSinceStart() <= 8000.0)
//       {
//         fnWriteLED(LED_RED);
//         is_able_to_pass_ = false;
//         // is_started[1] = false;
//       }
//       // else if (fnGetTimeSinceStart() > 8000.0)
//       // {
//       //   fnWriteLED(LED_GREEN);
//       //   is_able_to_pass_ = true;
//       //   // is_started[1] = false;
//       // }
//     }
//     // digitalWrite(3, HIGH);
//     // digitalWrite(5, LOW);
//     // digitalWrite(6, LOW);
//   }
//   else if (state == MUST_STOP)
//   {
//     // if (is_able_to_pass_ == false)
//     // {
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
//           fnWriteLED(LED_GREEN);
//           is_able_to_pass_ = true;
//           // is_started[1] = false;
//         }
//         else
//         {
//           fnWriteLED(LED_RED);
//           is_able_to_pass_ = false;
//           // is_started[1] = false;
//         }
//       }
//     // }
//     // digitalWrite(3, LOW);
//     // digitalWrite(5, HIGH);
//     // digitalWrite(6, LOW);
//
//   }
//   else if (state == IS_STOPPED)
//   {
//     if (is_able_to_pass_ == false)
//     {
//       fnWriteLED(ALL);
//       // fnSetStopWatch();
//       // is_started[1] = true;
//     }
//     else
//     {
//       // if (fnGetTimeSinceStart() > 50000.0)
//       // {
//         fnWriteLED(LED_GREEN);
//         // is_able_to_pass_ = true;
//         // is_started[1] = false;
//       // }
//     }
//
//     // is_able_to_pass_ = false;
//     // digitalWrite(3, LOW);
//     // digitalWrite(5, LOW);
//     // digitalWrite(6, HIGH);
//     // fnWriteLED(LED_GREEN);
//   }
//
//   Serial.print(fnGetTimeSinceStart());
//   Serial.println(" ");
// }
//
// void fnWriteLED(Color led_color)
// {
//   if (led_color == LED_RED)
//   {
//     digitalWrite(9, HIGH);
//     digitalWrite(10, LOW);
//     digitalWrite(11, LOW);
//   }
//   else if (led_color == LED_YELLOW)
//   {
//     digitalWrite(9, LOW);
//     digitalWrite(10, HIGH);
//     digitalWrite(11, LOW);
//   }
//   else if (led_color == LED_GREEN)
//   {
//     digitalWrite(9, LOW);
//     digitalWrite(10, LOW);
//     digitalWrite(11, HIGH);
//   }
//   else
//   {
//     digitalWrite(9, HIGH);
//     digitalWrite(10, HIGH);
//     digitalWrite(11, HIGH);
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
