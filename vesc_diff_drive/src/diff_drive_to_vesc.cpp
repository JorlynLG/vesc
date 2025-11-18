#include "vesc_diff_drive/diff_drive_to_vesc.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <vesc_msgs/msg/float64_stamped.hpp>

#include <cmath>
#include <sstream>
#include <string>

namespace vesc_diff_drive
{

using geometry_msgs::msg::TwistStamped;
using std::placeholders::_1;
using vesc_msgs::msg::Float64Stamped;


DiffDriveToVesc::DiffDriveToVesc(const rclcpp::NodeOptions & options)
:   Node("diff_drive_to_vesc_node", options)
{

    // declare parameters
    declare_parameter("wheel_separation", 0.0);
    declare_parameter("wheel_radius", 0.0);
    declare_parameter("rpm_to_ERPM", 0.0);
    declare_parameter("gear_ratio", 0.0);

    // get conversion parameters
    wheel_separation_ = get_parameter("wheel_separation").get_value<double>();
    wheel_radius_ = get_parameter("wheel_radius").get_value<double>();
    rpm_to_ERPM_ = get_parameter("rpm_to_ERPM").get_value<double>();
    gear_ratio_ = get_parameter("gear_ratio").get_value<double>();
    speed_conversion_ = rpm_to_ERPM_ * gear_ratio_ * 60 / (2 * M_PI * wheel_radius_);

    // create publishers to vesc electric-RPM (speed) and servo commands
    left_motor_pub_ = create_publisher<Float64Stamped>("left_motor/commands/motor/speed", 10);
    right_motor_pub_ = create_publisher<Float64Stamped>("right_motor/commands/motor/speed", 10);

    // subscribe to ackermann topic
    twist_sub_ = create_subscription<TwistStamped>(
        "cmd_vel", 10, std::bind(&DiffDriveToVesc::diffDriveCmdCallback, this, _1));
}

void DiffDriveToVesc::diffDriveCmdCallback(const TwistStamped::SharedPtr cmd)
{
    vesc_msgs::msg::Float64Stamped left_motor_msg;
    vesc_msgs::msg::Float64Stamped right_motor_msg;

    // Add timestamps
    left_motor_msg.header.stamp = this->now();
    right_motor_msg.header.stamp = this->now();

    // calc vesc electric RPM (speed)
    left_motor_msg.data = speed_conversion_*(cmd->twist.linear.x - cmd->twist.angular.z*wheel_separation_/2.0);
    right_motor_msg.data = speed_conversion_*(cmd->twist.linear.x + cmd->twist.angular.z*wheel_separation_/2.0);

    // publish
    if (rclcpp::ok()) {
        left_motor_pub_->publish(left_motor_msg);
        right_motor_pub_->publish(right_motor_msg);
    }
}

}  // namespace vesc_diff_drive

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_diff_drive::DiffDriveToVesc)
