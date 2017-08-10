cmake_minimum_required(VERSION 2.8.3)
project(rbiz_autorace)

find_package(catkin REQUIRED COMPONENTS
  roscpp
#  geometry_msgs
  rbiz_autorace_msgs
  genmsg
)

#add_message_files(DIRECTORY msg
#  FILES
#  SensorStateLevelCrossing.msg
#  InitStateLevelCrossing.msg
#  PadOrder.msg
#  AvailableItemList.msg
#  ServiceStatus.msg
#)

#add_service_files(DIRECTORY srv
#  FILES
#  InitTurtlebotPose.srv
#)

generate_messages(
  DEPENDENCIES
#  geometry_msgs
)


catkin_package(
# INCLUDE_DIRS include
# LIBRARIES key_to_vel_pkg
# CATKIN_DEPENDS roscpp rospy
# DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(monitor_traffic_light src/monitor_traffic_light.cpp)
target_link_libraries(monitor_traffic_light ${catkin_LIBRARIES})
add_dependencies(monitor_traffic_light rbiz_autorace_generate_messages_cpp)

add_executable(monitor_level_crossing src/monitor_level_crossing.cpp)
target_link_libraries(monitor_level_crossing ${catkin_LIBRARIES})
add_dependencies(monitor_level_crossing rbiz_autorace_generate_messages_cpp)
