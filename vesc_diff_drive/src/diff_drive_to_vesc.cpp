
#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <geometry_msgs/msg/TwistStamped.hpp>

#include <cmath>
#include <sstream>
#include <string>

namespace vesc_diff_drive
{

using geometry_msgs::msg::TwistStamped;


DiffDriveToVesc::DiffDriveToVesc(const rclcpp::NodeOptions & options)
:   Node("diff_drive_to_vesc_node", options)
{

    // create publishers to vesc electric-RPM (speed) and servo commands
    cmd_vel = create_publisher<TwistStamped>("cmd_vel", 10);
}

void DiffDriveToVesc::diffDriveCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
    // calc vesc electric RPM (speed)
    Float64 erpm_msg;
    erpm_msg.data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

    // publish
    if (rclcpp::ok()) {
        cmd_vel->publish(cmd_msg);
    }
}

}  // namespace vesc_diff_drive

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_diff_drive::DiffDriveToVesc)
