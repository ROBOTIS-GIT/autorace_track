#include <OLLO.h>
#include "motor_driver.h"

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <rbiz_autorace_msgs/SensorStateLevelCrossing.h>
#include <rbiz_autorace_msgs/DoIt.h>

/*******************************************************************************
* Definitions
*******************************************************************************/
#define DISTANCE_THRESHOLD_PASS         500

#define DXL_ID                          1
#define DXL_POSITION_VALUE_CLOSED       3072              // Dynamixel will rotate between this value
#define DXL_POSITION_VALUE_OPENED       2048              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_POSITION_VALUE_MIDDLE       2560              // (this is just for the test)

/*******************************************************************************
* Classes
*******************************************************************************/
ros::NodeHandle nh;

OLLO ollo;
MotorDriver motorDriver;

/*******************************************************************************
* Variables
*******************************************************************************/
// static uint16_t sensor_distance[3] = {0, 0, 0};
static uint16_t sensor_distance[3];// = {SENSOR_DISTANCE_INIT_VALUE, SENSOR_DISTANCE_INIT_VALUE, SENSOR_DISTANCE_INIT_VALUE};

double stopwatch_start_time_;
bool is_started[3];
bool is_able_to_pass_;

/*******************************************************************************
* Enum
*******************************************************************************/
static enum State { WAITING_FOR_ENTER, ENTERED, MUST_STOP, PASSED } vehicle_state_;
static enum Level { LEVEL_CLOSED, LEVEL_OPENED, LEVEL_MIDDLE } level_status_;
static enum Mode  { ACTIVE_MODE, TEST_MODE } mode_;

/*******************************************************************************
* Functions
*******************************************************************************/
void fnGetButtonPressed();
void fnReceiveSensorDistance();
void fnCheckVehicleStatus();
void fnLevelControl();
void fnInitStateLevelCrossing();
void fnTestStateLevelCrossing();


double fnGetCurrentTime();
double fnGetTimeSinceStart();
void fnSetStopWatch();

float fncheckVoltage();

/*******************************************************************************
* Publish function
*******************************************************************************/
void pbSensorState();

/*******************************************************************************
* Callback function for InitStateLevelCrossing msg
*******************************************************************************/
void cbInitStateLevelCrossing(const rbiz_autorace_msgs::DoIt& msgDoInitStateLevelCrossing);
void cbTestStateLevelCrossing(const rbiz_autorace_msgs::DoIt& msgTestStateLevelCrossing);
