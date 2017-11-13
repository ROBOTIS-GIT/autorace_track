#include <OLLO.h>

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <rbiz_autorace_msgs/SensorStateStopwatch.h>
#include <rbiz_autorace_msgs/DoIt.h>

/*******************************************************************************
* Definitions
*******************************************************************************/
#define DISTANCE_THRESHOLD_PASS         650

// #define DXL_ID                          1
// #define DXL_POSITION_VALUE_CLOSED       3072              // Dynamixel will rotate between this value
// #define DXL_POSITION_VALUE_OPENED       2048              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
// #define DXL_POSITION_VALUE_MIDDLE       2560              // (this is just for the test)

/*******************************************************************************
* Classes
*******************************************************************************/
ros::NodeHandle nh;

OLLO ollo;
// MotorDriver motorDriver;

/*******************************************************************************
* Variables
*******************************************************************************/
// static uint16_t sensor_distance[3] = {0, 0, 0};
static uint16_t sensor_distance[3];// = {SENSOR_DISTANCE_INIT_VALUE, SENSOR_DISTANCE_INIT_VALUE, SENSOR_DISTANCE_INIT_VALUE};

double stopwatch_start_time_;
double lap_time;
bool is_started[3];
bool is_finished;

// uint8_t led_turn_ = 1;

/*******************************************************************************
* Enum
*******************************************************************************/
static enum State { WAITING_FOR_START, STARTED, IS_GOING_TOWARD_RIGHT_DIRECTION, IS_ABOUT_TO_FINISH, FINISHED } vehicle_state_;
// static enum Color { LED_RED, LED_YELLOW, LED_GREEN, LED_ALL_LOW } led_color_;
static enum Mode  { ACTIVE_MODE, TEST_MODE } mode_;

/*******************************************************************************
* Functions
*******************************************************************************/
void fnGetButtonPressed();
void fnReceiveSensorDistance();
void fnCheckVehicleStatus();
void fnStopwatchControl();
void fnInitStateStopwatch();
void fnTestStateStopwatch();


double fnGetCurrentTime();
double fnGetTimeSinceStart();
void fnSetStopWatch();
void fnGetLapTime();

float fncheckVoltage();

/*******************************************************************************
* Publish function
*******************************************************************************/
void pbSensorState();

/*******************************************************************************
* Callback function for InitStateStopwatch msg
*******************************************************************************/
void cbInitStateStopwatch(const rbiz_autorace_msgs::DoIt& msgDoInitStateStopwatch);
void cbTestStateStopwatch(const rbiz_autorace_msgs::DoIt& msgTestStateStopwatch);
