cmake_minimum_required(VERSION 2.8.3)
project(costmap_cspace_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
)

add_message_files(
	FILES
    CSpace3D.msg
    CSpace3DUpdate.msg
    MapMetaData3D.msg
)
generate_messages(
	DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    std_msgs
)
