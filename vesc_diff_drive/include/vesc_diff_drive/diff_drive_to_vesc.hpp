#ifndef VESC_DIFF_DRIVE__DIFF_DRIVE_TO_VESC_HPP_
#define VESC_DIFF_DRIVE__DIFF_DRIVE_TO_VESC_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace vesc_diff_drive
{

using geometry_msgs::msg::Twist;
using std_msgs::msg::Float64;

class DiffDriveToVesc : public rclcpp::Node
{
public:
    explicit DiffDriveToVesc(const rclcpp::NodeOptions & options);

private:
    // ROS parameters
    double wheel_separation_;
    double wheel_radius_;
    double gear_ratio_;
    double rpm_to_ERPM_;
    double speed_conversion_;

    // ROS publishers + subscribers
    rclcpp::Publisher<Float64>::SharedPtr left_motor_pub_;
    rclcpp::Publisher<Float64>::SharedPtr right_motor_pub_;
    rclcpp::Subscription<Twist>::SharedPtr twist_sub_;

    // ROS callbacks
    void diffDriveCmdCallback(const Twist::SharedPtr cmd_vel);
};

}  // namespace vesc_diff_drive

#endif  // VESC_DIFF_DRIVE__DIFF_DRIVE_TO_VESC_HPP_
