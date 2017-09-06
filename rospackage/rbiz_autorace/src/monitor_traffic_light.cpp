#include "ros/ros.h"
#include "rbiz_autorace_msgs/SensorStateTrafficLight.h"
#include "rbiz_autorace_msgs/DoIt.h"

class MonitorTrafficLight
{
public:
  MonitorTrafficLight()
  {
    // fnInitParam();

    pubInitStateTrafficLight = nh_.advertise<rbiz_autorace_msgs::DoIt>("init_state/traffic_light", 1);
    pubTestStateTrafficLight = nh_.advertise<rbiz_autorace_msgs::DoIt>("test_state/traffic_light", 1);

    subSensorStateTrafficLight = nh_.subscribe("sensor_state/traffic_light", 1, &MonitorTrafficLight::cbReceiveSensorStateTrafficLight, this);

    ros::Rate loop_rate(10);

    loop_rate.sleep();

    // Publish Init State Traffic Light
    // pbInitStateTrafficLight();

    // Publish Test State Traffic Light
    pbTestStateTrafficLight();

    // Loop
    while (ros::ok())
    {
      // pbInitStateLevelCrossing();
      // fnPubServiceStatus();

      // fnPubPose();
      ros::spinOnce();
      loop_rate.sleep();
    }
    // is_pose_initialized = fnSetInitialPose();
  }

  // Callback Functions
  void cbReceiveSensorStateTrafficLight(const rbiz_autorace_msgs::SensorStateTrafficLight msgSensorStateTrafficLight)
  {
    ROS_INFO("%d", msgSensorStateTrafficLight.sensor_distance[0]);
  }

  //
  // void fnInitParam()
  // {
  //   nh_.getParam("table_pose_tb3p/position", target_pose_position);
  //   nh_.getParam("table_pose_tb3p/orientation", target_pose_orientation);
  //
  //   poseStampedTable[0].header.frame_id = "map";
  //   poseStampedTable[0].header.stamp = ros::Time::now();
  //
  //   poseStampedTable[0].pose.position.x = target_pose_position[0];
  //   poseStampedTable[0].pose.position.y = target_pose_position[1];
  //   poseStampedTable[0].pose.position.z = target_pose_position[2];
  //
  //   poseStampedTable[0].pose.orientation.x = target_pose_orientation[0];
  //   poseStampedTable[0].pose.orientation.y = target_pose_orientation[1];
  //   poseStampedTable[0].pose.orientation.z = target_pose_orientation[2];
  //   poseStampedTable[0].pose.orientation.w = target_pose_orientation[3];
  //
  //
  //   nh_.getParam("table_pose_tb3g/position", target_pose_position);
  //   nh_.getParam("table_pose_tb3g/orientation", target_pose_orientation);
  //
  //   poseStampedTable[1].header.frame_id = "map";
  //   poseStampedTable[1].header.stamp = ros::Time::now();
  //
  //   poseStampedTable[1].pose.position.x = target_pose_position[0];
  //   poseStampedTable[1].pose.position.y = target_pose_position[1];
  //   poseStampedTable[1].pose.position.z = target_pose_position[2];
  //
  //   poseStampedTable[1].pose.orientation.x = target_pose_orientation[0];
  //   poseStampedTable[1].pose.orientation.y = target_pose_orientation[1];
  //   poseStampedTable[1].pose.orientation.z = target_pose_orientation[2];
  //   poseStampedTable[1].pose.orientation.w = target_pose_orientation[3];
  //
  //
  //   nh_.getParam("table_pose_tb3r/position", target_pose_position);
  //   nh_.getParam("table_pose_tb3r/orientation", target_pose_orientation);
  //
  //   poseStampedTable[2].header.frame_id = "map";
  //   poseStampedTable[2].header.stamp = ros::Time::now();
  //
  //   poseStampedTable[2].pose.position.x = target_pose_position[0];
  //   poseStampedTable[2].pose.position.y = target_pose_position[1];
  //   poseStampedTable[2].pose.position.z = target_pose_position[2];
  //
  //   poseStampedTable[2].pose.orientation.x = target_pose_orientation[0];
  //   poseStampedTable[2].pose.orientation.y = target_pose_orientation[1];
  //   poseStampedTable[2].pose.orientation.z = target_pose_orientation[2];
  //   poseStampedTable[2].pose.orientation.w = target_pose_orientation[3];
  //
  //
  //   nh_.getParam("counter_pose_bread/position", target_pose_position);
  //   nh_.getParam("counter_pose_bread/orientation", target_pose_orientation);
  //
  //   poseStampedCounter[0].header.frame_id = "map";
  //   poseStampedCounter[0].header.stamp = ros::Time::now();
  //
  //   poseStampedCounter[0].pose.position.x = target_pose_position[0];
  //   poseStampedCounter[0].pose.position.y = target_pose_position[1];
  //   poseStampedCounter[0].pose.position.z = target_pose_position[2];
  //
  //   poseStampedCounter[0].pose.orientation.x = target_pose_orientation[0];
  //   poseStampedCounter[0].pose.orientation.y = target_pose_orientation[1];
  //   poseStampedCounter[0].pose.orientation.z = target_pose_orientation[2];
  //   poseStampedCounter[0].pose.orientation.w = target_pose_orientation[3];
  //
  //
  //   nh_.getParam("counter_pose_drink/position", target_pose_position);
  //   nh_.getParam("counter_pose_drink/orientation", target_pose_orientation);
  //
  //   poseStampedCounter[1].header.frame_id = "map";
  //   poseStampedCounter[1].header.stamp = ros::Time::now();
  //
  //   poseStampedCounter[1].pose.position.x = target_pose_position[0];
  //   poseStampedCounter[1].pose.position.y = target_pose_position[1];
  //   poseStampedCounter[1].pose.position.z = target_pose_position[2];
  //
  //   poseStampedCounter[1].pose.orientation.x = target_pose_orientation[0];
  //   poseStampedCounter[1].pose.orientation.y = target_pose_orientation[1];
  //   poseStampedCounter[1].pose.orientation.z = target_pose_orientation[2];
  //   poseStampedCounter[1].pose.orientation.w = target_pose_orientation[3];
  //
  //
  //   nh_.getParam("counter_pose_snack/position", target_pose_position);
  //   nh_.getParam("counter_pose_snack/orientation", target_pose_orientation);
  //
  //   poseStampedCounter[2].header.frame_id = "map";
  //   poseStampedCounter[2].header.stamp = ros::Time::now();
  //
  //   poseStampedCounter[2].pose.position.x = target_pose_position[0];
  //   poseStampedCounter[2].pose.position.y = target_pose_position[1];
  //   poseStampedCounter[2].pose.position.z = target_pose_position[2];
  //
  //   poseStampedCounter[2].pose.orientation.x = target_pose_orientation[0];
  //   poseStampedCounter[2].pose.orientation.y = target_pose_orientation[1];
  //   poseStampedCounter[2].pose.orientation.z = target_pose_orientation[2];
  //   poseStampedCounter[2].pose.orientation.w = target_pose_orientation[3];
  // }

  // Publish Functions
  void pbInitStateTrafficLight()
  {
    msgDoInitStateTrafficLight.doIt = true;

    pubInitStateTrafficLight.publish(msgDoInitStateTrafficLight);
  }

  void pbTestStateTrafficLight()
  {
    msgDoTestStateTrafficLight.doIt = true;

    pubTestStateTrafficLight.publish(msgDoTestStateTrafficLight);
  }

private:

  ros::NodeHandle nh_;

  // Publisher
  ros::Publisher pubInitStateTrafficLight;
  ros::Publisher pubTestStateTrafficLight;
  // ros::Publisher pubServiceStatusPadtb3p;
  // ros::Publisher pubServiceStatusPadtb3g;
  // ros::Publisher pubServiceStatusPadtb3r;
  // ros::Publisher pub_is_item_available;
  //
  // ros::Publisher pub_play_sound_tb3p;
  // ros::Publisher pub_play_sound_tb3g;
  // ros::Publisher pub_play_sound_tb3r;
  //
  // ros::Publisher pubPoseStampedTb3p;
  // ros::Publisher pubPoseStampedTb3g;
  // ros::Publisher pubPoseStampedTb3r;

  // Subscriber
  ros::Subscriber subSensorStateTrafficLight;
  // ros::Subscriber sub_pad_order_tb3r;
  // ros::Subscriber sub_pad_order_tb3p;

  // ros::Subscriber sub_arrival_status_tb3p;
  // ros::Subscriber sub_arrival_status_tb3g;
  // ros::Subscriber sub_arrival_status_tb3r;

  // Messages
  rbiz_autorace_msgs::DoIt msgDoInitStateTrafficLight;
  rbiz_autorace_msgs::DoIt msgDoTestStateTrafficLight;
  // geometry_msgs::PoseStamped poseStampedTable[3];
  // geometry_msgs::PoseStamped poseStampedCounter[3];

  // std::vector<double> target_pose_position;
  // std::vector<double> target_pose_orientation;
  //
  // boost::array<int, 3> item_num_chosen_by_pad = { {-1, -1, -1} };
  // boost::array<int, 3> is_item_available = { {1, 1, 1} };
  // boost::array<int, 3> robot_service_sequence = { {0, 0, 0} };
  //
  // bool is_robot_reached_target[3] = {true, true, true};
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "traffic_light_monitor");

  //Create an object of class ServiceCore that will take care of everything
  MonitorTrafficLight trafficLightMonitor;

  ros::spin();

  return 0;
}
