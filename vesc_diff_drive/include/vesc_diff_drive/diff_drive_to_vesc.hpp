#ifndef VESC_DIFF_DRIVE__DIFF_DRIVE_TO_VESC_HPP_
#define VESC_DIFF_DRIVE__DIFF_DRIVE_TO_VESC_HPP_

#include <geometry_msgs/msg/TwistStamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace vesc_diff_drive
{

using geometry_msgs::msg::TwistStamped;

class DiffDriveToVesc : public rclcpp::Node
{
public:
    explicit DiffDriveToVesc(const rclcpp::NodeOptions & options);

private:
  // ROS parameters
    char* left_wheel_names
    char* right_wheel_names
    double wheel_separation
    int wheels_per_side
    double wheel_radius
    bool odom_loop

  // ROS publishers
    rclcpp::Publisher<TwistStamped>::SharedPtr cmd_vel;

  // ROS callbacks
    void diffDriveCmdCallback(const TwistStamped::SharedPtr cmd_vel);
};

}  // namespace vesc_diff_drive

#endif  // VESC_DIFF_DRIVE__DIFF_DRIVE_TO_VESC_HPP_
