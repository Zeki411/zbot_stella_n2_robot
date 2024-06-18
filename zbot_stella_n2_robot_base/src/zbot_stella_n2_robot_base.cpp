#include "zbot_stella_n2_robot_base.hpp"
#include "MW_serial.hpp"
#include "MW_value.hpp"
#include "stella.hpp"

static bool RUN = false;

zbot_stella_n2_robot_base_node::zbot_stella_n2_robot_base_node() : Node("zbot_stella_n2_robot_base_node")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&zbot_stella_n2_robot_base_node::command_velocity_callback, this, std::placeholders::_1));
    // odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
    // odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    serial_timer = this->create_wall_timer(1ms, std::bind(&zbot_stella_n2_robot_base_node::serial_timer_callback, this));
}

zbot_stella_n2_robot_base_node::~zbot_stella_n2_robot_base_node()
{
  RUN = false;
  MW_Serial_DisConnect();
}

void zbot_stella_n2_robot_base_node::command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg) {
  if(RUN) {
    goal_linear_velocity_ = cmd_vel_msg->linear.x;
    goal_angular_velocity_ = cmd_vel_msg->angular.z * -1.0;

    dual_m_command(dual_m_command_select::m_lav, goal_linear_velocity_, goal_angular_velocity_);
  }
}

void zbot_stella_n2_robot_base_node::serial_timer_callback() {
  if(RUN) {
    Motor_MonitoringCommand(channel_1, _position);
    Motor_MonitoringCommand(channel_2, _position);

    update_odometry();
  }
}

void zbot_stella_n2_robot_base_node::update_odometry() {
  // Update odometry
  // ...
}

int main(int argc, char **argv){
  rclcpp::init(argc, argv);

  MW_Serial_Connect("/dev/MW", 115200);

  if(Robot_Setting(::N2)) RUN = true;
  Robot_Fault_Checking_RESET();
  
  rclcpp::spin(std::make_shared<zbot_stella_n2_robot_base_node>());

  rclcpp::shutdown();
  return 0;
}