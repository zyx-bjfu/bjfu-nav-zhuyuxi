#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <chrono>

class ChassisNode : public rclcpp::Node
{
public:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double pose_x_ = 0.0;
  double pose_y_ = 0.0;
  double pose_theta_ = 0.0;
  double vx_ = 0.0;
  double omega_ = 0.0;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;
  rclcpp::Time last_cmd_time_;

  ChassisNode() : Node("chassis_Node")
  {
    RCLCPP_INFO(this->get_logger(), "ChassisNode 启动了");

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&ChassisNode::cmd_vel_callback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    last_time_ = this->now();
    last_cmd_time_ = this->now();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&ChassisNode::timer_callback, this));
  }

  void timer_callback();
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
};

void ChassisNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  vx_ = msg->linear.x;
  omega_ = msg->angular.z;
  last_cmd_time_ = this->now();
}

void ChassisNode::timer_callback()
{
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;

  if (dt <= 0.0 || dt > 0.5) {
    dt = 0.0;
  }

  if ((current_time - last_cmd_time_).seconds() > 0.2) {
    vx_ = 0.0;
    omega_ = 0.0;
  }

  pose_theta_ += omega_ * dt;
  pose_x_ += vx_ * cos(pose_theta_) * dt;
  pose_y_ += vx_ * sin(pose_theta_) * dt;

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  odom_msg.header.stamp = current_time;
  odom_msg.pose.pose.position.x = pose_x_;
  odom_msg.pose.pose.position.y = pose_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(pose_theta_ / 2);
  odom_msg.pose.pose.orientation.w = cos(pose_theta_ / 2);
  odom_msg.twist.twist.linear.x = vx_;
  odom_msg.twist.twist.angular.z = omega_;

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = current_time;
  tf_msg.header.frame_id = "odom";
  tf_msg.child_frame_id = "base_link";
  tf_msg.transform.translation.x = pose_x_;
  tf_msg.transform.translation.y = pose_y_;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation.x = 0.0;
  tf_msg.transform.rotation.y = 0.0;
  tf_msg.transform.rotation.z = sin(pose_theta_ / 2);
  tf_msg.transform.rotation.w = cos(pose_theta_ / 2);

  tf_broadcaster_->sendTransform(tf_msg);
  odom_pub_->publish(odom_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisNode>());
  rclcpp::shutdown();
  return 0;
}
