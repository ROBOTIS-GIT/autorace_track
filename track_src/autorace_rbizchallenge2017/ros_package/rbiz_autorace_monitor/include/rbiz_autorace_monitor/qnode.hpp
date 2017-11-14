/**
 * @file /include/rbiz_autorace_monitor/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef rbiz_autorace_monitor_QNODE_HPP_
#define rbiz_autorace_monitor_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rbiz_autorace_msgs/SensorStateTrafficLight.h>
#include <rbiz_autorace_msgs/SensorStateLevelCrossing.h>
#include <rbiz_autorace_msgs/SensorStateStopwatch.h>
#include <rbiz_autorace_msgs/DoIt.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rbiz_autorace_monitor {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

  enum StopwatchStatus {
    STAY,
    STARTED,
    FINISHED
  } stopwatchStatus;

  double lap_time = 0.0;


  // explicit MyClass(bool willSleep, QString name, QObject *parent = 0);
  // void updateCount();
  // QTimer *timer;
  // int count;
  // bool m_wantToSleep;


	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

  void pbInitStateTrafficLight();
  void pbInitStateLevelCrossing();
  void pbInitStateStopwatch();

  void pbTestStateTrafficLight();
  void pbTestStateLevelCrossing();

  void cbReceiveSensorStateTrafficLight(const rbiz_autorace_msgs::SensorStateTrafficLight msgSensorStateTrafficLight);
  void cbReceiveSensorStateLevelCrossing(const rbiz_autorace_msgs::SensorStateLevelCrossing msgSensorStateLevelCrossing);
  void cbReceiveSensorStateStopwatch(const rbiz_autorace_msgs::SensorStateStopwatch msgSensorStateStopwatch);


	// QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
  void fnUpdateLapTime();
// void fnPassLapTime();
	// void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;

  // Publisher
  ros::Publisher pubInitStateTrafficLight;
  ros::Publisher pubInitStateLevelCrossing;
  ros::Publisher pubInitStateStopwatch;

  ros::Publisher pubTestStateTrafficLight;
  ros::Publisher pubTestStateLevelCrossing;

  // Subscriber
  ros::Subscriber subSensorStateTrafficLight;

  ros::Subscriber subSensorStateLevelCrossing;

  ros::Subscriber subSensorStateStopwatch;


  // Messages
  rbiz_autorace_msgs::DoIt msgDoInitStateTrafficLight;
  rbiz_autorace_msgs::DoIt msgDoInitStateLevelCrossing;
  rbiz_autorace_msgs::DoIt msgDoInitStateStopwatch;

  rbiz_autorace_msgs::DoIt msgDoTestStateTrafficLight;
  rbiz_autorace_msgs::DoIt msgDoTestStateLevelCrossing;


  // QStringListModel logging_model;
};

}  // namespace rbiz_autorace_monitor

#endif /* rbiz_autorace_monitor_QNODE_HPP_ */
