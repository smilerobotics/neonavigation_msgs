cmake_minimum_required(VERSION 2.8.3)
project(trajectory_tracker_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
)

add_service_files(
  FILES
    ChangePath.srv
)
add_message_files(
  FILES
    TrajectoryTrackerStatus.msg
    TrajectoryServerStatus.msg
)
generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  DEPENDS
    std_msgs
)
