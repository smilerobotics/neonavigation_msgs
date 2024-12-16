cmake_minimum_required(VERSION 3.5)
project(neonavigation_metrics_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/Metrics.msg
        msg/Metric.msg
    DEPENDENCIES ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
)

if(BUILD_TESTING)
    set(ament_cmake_copyright_FOUND TRUE)
    ament_auto_find_test_dependencies()
    ament_lint_auto_find_test_dependencies()

    ament_auto_add_gtest(test_helper test/src/test_helper.cpp)
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(test_helper "${cpp_typesupport_target}")
endif()

ament_auto_package()
