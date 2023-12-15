// C++ header
#include <string>
#include <memory>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


namespace trajectory_server
{
class TrajectoryServer : public rclcpp::Node
{
public:
  TrajectoryServer();
  ~TrajectoryServer() = default;

private:
  void waitForTf();
  void addCurrentTfPoseToTrajectory();
  void trajectoryUpdateTimerCallback();
  void publishTrajectoryTimerCallback();

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string p_target_frame_name_;
  std::string p_source_frame_name_;
  double p_trajectory_update_rate_;
  double p_trajectory_publish_rate_;

  rclcpp::TimerBase::SharedPtr update_trajectory_timer_;
  rclcpp::TimerBase::SharedPtr publish_trajectory_timer_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;

  // Zero pose used for transformation to target_frame.
  geometry_msgs::msg::PoseStamped pose_source_;

  nav_msgs::msg::Path trajectory_;
};

}
