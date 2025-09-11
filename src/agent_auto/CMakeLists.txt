cmake_minimum_required(VERSION 3.5)
project(agent_auto)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(multi_agent_planner_msgs REQUIRED)

include_directories(include ${Eigen3_INCLUDE_DIRS})
add_executable(agent_auto
        src/main.cpp)
ament_target_dependencies(agent_auto 
  rclcpp 
  px4_ros2_cpp 
  px4_ros_com
  multi_agent_planner_msgs
  Eigen3)

install(TARGETS
        agent_auto
        DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
