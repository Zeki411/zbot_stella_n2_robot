#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

class zbot_stella_n2_robot_base_node : public rclcpp::Node
{
public:
    zbot_stella_n2_robot_base_node();
    ~zbot_stella_n2_robot_base_node();

private:
    // ROS parameters
    bool odom_publish_tf_;
    std::string odom_frame_id_;
    std::string odom_child_frame_id;

    // ROS timer
    rclcpp::TimerBase::SharedPtr serial_timer;

    // ROS topic publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // ROS topic subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Time time_now;

    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

    double goal_linear_velocity_;
    double goal_angular_velocity_;

    int left_encoder_prev=0,right_encoder_prev=0;

    double delta_th=0.0,delta_s=0.0,delta_x=0.0,delta_y=0.0,x=0.0,y=0.0,th=0.0,delta_left = 0,delta_right = 0;

    void ahrs_yaw_data_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
    void serial_timer_callback();
    void update_odometry();
};