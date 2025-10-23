#include "vesc_diff_drive/vesc_to_odom.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include <cmath>
#include <string>

namespace vesc_diff_drive
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
:   Node("vesc_to_odom_node", options),
    odom_frame_("odom"),
    base_frame_("base_link"),
    x_(0.0),
    y_(0.0),
    yaw_(0.0), 
    speed_left_(0.0),
    speed_right_(0.0),
    last_time_(this->now())
{
    // get ROS parameters
    odom_frame_ = declare_parameter("odom_frame", odom_frame_);
    base_frame_ = declare_parameter("base_frame", base_frame_);
  
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

    // create odom publisher
    odom_pub_ = create_publisher<Odometry>("odom", 10);

    // create tf broadcaster
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));

    // subscribe to vesc state
    vesc_state_left_sub_ = create_subscription<VescStateStamped>(
        "left_motor/sensors/core", 10, std::bind(&VescToOdom::vescStateCallbackLeft, this, _1));
    vesc_state_right_sub_ = create_subscription<VescStateStamped>(
        "right_motor/sensors/core", 10, std::bind(&VescToOdom::vescStateCallbackRight, this, _1));
    odom_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),  // Call every 20ms (50Hz)
        std::bind(&VescToOdom::publishOdom, this));
}

void VescToOdom::vescStateCallbackLeft(const VescStateStamped::SharedPtr state)
{
    // convert to engineering units
    speed_left_ = (state->state.speed)/speed_conversion_;
    if (std::fabs(speed_left_) < 0.05) {
        speed_left_ = 0.0;
    }

    // use current state as last state if this is our first time here
    if (!last_state_left_) {
        last_state_left_ = state;
    }
}

void VescToOdom::vescStateCallbackRight(const VescStateStamped::SharedPtr state)
{
    // convert to engineering units
    speed_right_ = (state->state.speed)/speed_conversion_;
    if (std::fabs(speed_right_) < 0.05) {
        speed_right_ = 0.0;
    }
}

void VescToOdom::publishOdom()
{
    // calc elapsed time
    auto now = this->now();
    double dt = (now - last_time_).seconds();

    double current_speed = (speed_right_+ speed_left_) / 2;
    double current_angular_velocity = (speed_right_ - speed_left_) / wheel_separation_;

    // propigate odometry
    double x_dot = current_speed * cos(yaw_);
    double y_dot = current_speed * sin(yaw_);
    x_ += x_dot * dt;
    y_ += y_dot * dt;
    yaw_ += current_angular_velocity * dt;

    // publish odometry message
    Odometry odom;
    odom.header.frame_id = odom_frame_;
    odom.header.stamp = now;
    odom.child_frame_id = base_frame_;

    // Position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = sin(yaw_ / 2.0);
    odom.pose.pose.orientation.w = cos(yaw_ / 2.0);

    // Position uncertainty
    /** @todo Think about position uncertainty, perhaps get from parameters? */
    odom.pose.covariance[0] = 0.2;   ///< x
    odom.pose.covariance[7] = 0.2;   ///< y
    odom.pose.covariance[35] = 0.4;  ///< yaw

    // Velocity ("in the coordinate frame given by the child_frame_id")
    odom.twist.twist.linear.x = current_speed;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = current_angular_velocity;

    // Velocity uncertainty
    /** @todo Think about velocity uncertainty */

    TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = now;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;

    if (rclcpp::ok()) {
        tf_pub_->sendTransform(tf);
        odom_pub_->publish(odom);
    }
    last_time_ = now;
}

}  // namespace vesc_diff_drive

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_diff_drive::VescToOdom)
