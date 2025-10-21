#ifndef VESC_DIFF_DRIVE__VESC_TO_ODOM_HPP_
#define VESC_DIFF_DRIVE__VESC_TO_ODOM_HPP_

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include <memory>
#include <string>

namespace vesc_diff_drive
{
using nav_msgs::msg::Odometry;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

class VescToOdom : public rclcpp::Node
{
public:
  explicit VescToOdom(const rclcpp::NodeOptions & options);

private:
    // ROS parameters
    std::string odom_frame_;
    std::string base_frame_;

    // conversion gain and offset
    // ROS parameters
    double wheel_separation_;
    double wheel_radius_;
    double gear_ratio_;
    double rpm_to_ERPM_;
    double speed_conversion_;

    // odometry state
    double x_, y_, yaw_;
    double speed_left_, speed_right_;
    VescStateStamped::SharedPtr last_state_left_;  ///< Last received state message
    VescStateStamped::SharedPtr last_state_right_;  ///< Last received state message
    rclcpp::Time last_time_;

    // ROS services
    rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_state_left_sub_;
    rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_state_right_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

    // ROS callbacks
    void vescStateCallbackLeft(const VescStateStamped::SharedPtr state);
    void vescStateCallbackRight(const VescStateStamped::SharedPtr state);
    rclcpp::TimerBase::SharedPtr odom_timer_;
};

}  // namespace vesc_diff_drive

#endif  // VESC_DIFF_DRIVE__VESC_TO_ODOM_HPP_
