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

include_directories(
  "${PROJECT_SOURCE_DIR}/lib/serial"
  "${PROJECT_SOURCE_DIR}/lib/mw_motor_driver"
  "${PROJECT_SOURCE_DIR}/lib/mw"
)



add_executable(${PROJECT_NAME}_node 
  src/zbot_stella_n2_robot_base.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  nav_msgs
)

target_link_libraries(${PROJECT_NAME}_node ${CMAKE_CURRENT_SOURCE_DIR}/lib/mw_motor_driver/MW_MotorDriver_aarch64.a)

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
