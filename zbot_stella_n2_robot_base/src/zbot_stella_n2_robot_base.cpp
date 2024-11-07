#include "zbot_stella_n2_robot_base.hpp"
#include "MW_serial.hpp"
#include "MW_value.hpp"
#include "stella.hpp"


#define convertor_d2r (M_PI / 180.0)

static bool RUN = false;

inline int Limit_i(int v, int lo, int hi) {
  // Check if the absolute value of v is within the specified range (exclusive)
  if (abs(v) > lo && abs(v) < hi) {
    return v; // Return v if within range
  } else {
    return 0; // Return 0 if outside the range
  }
}

inline double pulse2meter()
{
  double meter= ((2 * M_PI * Differential_MobileRobot.wheel_radius) / Differential_MobileRobot.gear_ratio / MyMotorConfiguration.encoder_ppr[0]);

  return meter; 
}

zbot_stella_n2_robot_base_node::zbot_stella_n2_robot_base_node() : Node("zbot_stella_n2_robot_base_node")
{   
    this->declare_parameter<bool>("odom_publish_tf", false);
    this->declare_parameter<std::string>("odom_frame_id", "odom"); 
    this->declare_parameter<std::string>("odom_child_frame_id", "base_footprint");

    this->get_parameter("odom_publish_tf", odom_publish_tf_);
    this->get_parameter("odom_frame_id", odom_frame_id_);
    this->get_parameter("odom_child_frame_id", odom_child_frame_id);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();

    // ahrs_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>("imu/yaw", 1, std::bind(&zbot_stella_n2_robot_base_node::ahrs_yaw_data_callback, this, std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&zbot_stella_n2_robot_base_node::command_velocity_callback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
    odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
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
  // Calculate the distance each wheel has traveled
  delta_left = Limit_i((MyMotorCommandReadValue.position[channel_1] - left_encoder_prev), 0, 15000) * pulse2meter();
  delta_right = Limit_i((MyMotorCommandReadValue.position[channel_2] - right_encoder_prev), 0, 15000) * pulse2meter();

  // Calculate the change in the robot's position and orientation
  delta_s  = (delta_right + delta_left) / 2.0;
  delta_th += -1.0 * (delta_right - delta_left) / Differential_MobileRobot.axle_length; // reverse bugs fixed by -1.0

  // Update the robot's position
  delta_x  = delta_s * cos(delta_th);
  delta_y  = delta_s * sin(delta_th);
  
  x += delta_x;
  y += delta_y;

  // Create the odometry message
  nav_msgs::msg::Odometry odom;

  // Calculate the robot's orientation
  tf2::Quaternion Quaternion;
  Quaternion.setRPY(0, 0, delta_th);

  // Set the orientation in the odometry message
  odom.pose.pose.orientation.x = Quaternion.x();
  odom.pose.pose.orientation.y = Quaternion.y();
  odom.pose.pose.orientation.z = Quaternion.z();
  odom.pose.pose.orientation.w = Quaternion.w();

  
  rclcpp::Time time_now = this->now();

  if(odom_publish_tf_) {
    // Create and send the transform

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = time_now;
    t.header.frame_id = odom_frame_id_;
    t.child_frame_id = odom_child_frame_id;

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odom.pose.pose.orientation;

    odom_broadcaster->sendTransform(t);
  }

  // Populate the rest of the odometry message
  odom.header.stamp = time_now;
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = odom_child_frame_id;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;

  // Calculate covariance for pose

  // Calculate the linear and angular velocities
  odom.twist.twist.linear.x = goal_linear_velocity_;
  odom.twist.twist.angular.z = goal_angular_velocity_;

  // Publish the odometry message
  odom_pub_->publish(odom);

  // Update previous encoder values
  left_encoder_prev = MyMotorCommandReadValue.position[channel_1];
  right_encoder_prev = MyMotorCommandReadValue.position[channel_2];
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