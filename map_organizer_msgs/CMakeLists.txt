cmake_minimum_required(VERSION 2.8.3)
project(map_organizer_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    nav_msgs
)

add_message_files(
  FILES
    OccupancyGridArray.msg
)
generate_messages(
  DEPENDENCIES
    nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    nav_msgs
)
