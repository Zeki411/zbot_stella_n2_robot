################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(zbot_stella_n2_robot_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)


include_directories(
  "${PROJECT_SOURCE_DIR}/lib/serial"
  "${PROJECT_SOURCE_DIR}/lib/mw_motor_driver"
  "${PROJECT_SOURCE_DIR}/lib/mw"
)

add_executable(base_node 
  src/zbot_stella_n2_robot_base.cpp
  lib/serial/list_ports_linux.cc 
  lib/serial/serial.cc 
  lib/serial/unix.cc 
  lib/mw/stella.cpp
)
ament_target_dependencies(base_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2_msgs"
  "tf2_ros"
  "tf2"
)

target_link_libraries(base_node ${CMAKE_CURRENT_SOURCE_DIR}/lib/mw_motor_driver/MW_MotorDriver_aarch64.a)

################################################################################
# Install
################################################################################
install(
  DIRECTORY 
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  base_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
