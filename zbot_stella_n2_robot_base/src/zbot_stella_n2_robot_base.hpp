#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class zbot_stella_n2_robot_base_node : public rclcpp::Node
{
public:
    zbot_stella_n2_robot_base_node();
    ~zbot_stella_n2_robot_base_node();

private:

    double goal_linear_velocity_;
    double goal_angular_velocity_;

    // ROS timer
    rclcpp::TimerBase::SharedPtr serial_timer;

    // ROS topic publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // ROS topic subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;


    void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
    void serial_timer_callback();
    void update_odometry();
};