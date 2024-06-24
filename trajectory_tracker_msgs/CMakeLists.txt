cmake_minimum_required(VERSION 3.5)
project(trajectory_tracker_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/TrajectoryServerStatus.msg
        msg/PoseStampedWithVelocity.msg
        msg/PathWithVelocity.msg
        msg/TrajectoryTrackerStatus.msg
        srv/ChangePath.srv
    DEPENDENCIES ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
)

if(BUILD_TESTING)
    find_package(Threads REQUIRED)
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    set(ament_cmake_copyright_FOUND TRUE)
    ament_auto_find_test_dependencies()
    ament_lint_auto_find_test_dependencies()

    ament_auto_add_gtest(test_path_with_velocity_conversion
        test/src/test_path_with_velocity_conversion.cpp)
    target_include_directories(test_path_with_velocity_conversion PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
    )
endif()

ament_auto_package()
