cmake_minimum_required(VERSION 2.8.3)
project(map_organizer_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    nav_msgs
    message_generation
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
  DEPENDS
    nav_msgs
)
