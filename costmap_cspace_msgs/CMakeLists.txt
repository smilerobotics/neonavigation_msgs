cmake_minimum_required(VERSION 3.5)
project(costmap_cspace_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/CSpace3D.msg
        msg/CSpace3DUpdate.msg
        msg/MapMetaData3D.msg
    DEPENDENCIES ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
)

if(BUILD_TESTING)
    set(ament_cmake_copyright_FOUND TRUE)
    ament_auto_find_test_dependencies()
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
