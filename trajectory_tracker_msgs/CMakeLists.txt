cmake_minimum_required(VERSION 3.5)
project(trajectory_tracker_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        action/FollowPathWithVelocity.action
        msg/TrajectoryServerStatus.msg
        msg/PoseStampedWithVelocity.msg
        msg/PathWithVelocity.msg
        msg/TrajectoryTrackerStatus.msg
        srv/ChangePath.srv
    DEPENDENCIES ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
)

if(BUILD_TESTING)
    set(ament_cmake_copyright_FOUND TRUE)
    ament_auto_find_test_dependencies()
    ament_lint_auto_find_test_dependencies()

    ament_auto_add_gtest(test_path_with_velocity_conversion
        test/src/test_path_with_velocity_conversion.cpp)
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(test_path_with_velocity_conversion "${cpp_typesupport_target}")
endif()

ament_auto_package()
