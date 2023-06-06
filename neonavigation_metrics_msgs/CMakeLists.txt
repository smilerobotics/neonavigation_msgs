cmake_minimum_required(VERSION 2.8.3)
project(neonavigation_metrics_msgs)

set(MESSAGE_DEPENDS
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS message_generation ${MESSAGE_DEPENDS})

add_message_files(
  FILES
    Metric.msg
    Metrics.msg
)
generate_messages(DEPENDENCIES ${MESSAGE_DEPENDS})
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  catkin_add_gtest(test_helper test/src/test_helper.cpp)
  target_link_libraries(test_helper ${catkin_LIBRARIES})

  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
