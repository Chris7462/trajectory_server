// ROS header
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// local Header
#include "trajectory_server/trajectory_server.hpp"


namespace trajectory_server
{

TrajectoryServer::TrajectoryServer()
: Node("trajectory_server"),
  tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  declare_parameter("target_frame_name", std::string("map"));
  declare_parameter("source_frame_name", std::string("base_link"));
  declare_parameter("trajectory_update_rate", 4.0);
  declare_parameter("trajectory_publish_rate", 0.25);

  p_target_frame_name_ = get_parameter("target_frame_name").as_string();
  p_source_frame_name_ = get_parameter("source_frame_name").as_string();
  p_trajectory_update_rate_ = get_parameter("trajectory_update_rate").as_double();
  p_trajectory_publish_rate_ = get_parameter("trajectory_publish_rate").as_double();

  waitForTf();

  update_trajectory_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / p_trajectory_update_rate_),
    std::bind(&TrajectoryServer::trajectoryUpdateTimerCallback, this));

  publish_trajectory_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / p_trajectory_publish_rate_),
    std::bind(&TrajectoryServer::publishTrajectoryTimerCallback, this));

  trajectory_pub_ = create_publisher<nav_msgs::msg::Path>("trajectory", 10);

  pose_source_.pose.orientation.w = 1.0;
  pose_source_.header.frame_id = p_source_frame_name_;

  trajectory_.header.frame_id = p_target_frame_name_;
}

void TrajectoryServer::waitForTf()
{
  rclcpp::Time start = rclcpp::Node::now();

  RCLCPP_INFO(
    get_logger(), "Waiting for tf transform data between frames %s and %s to become available",
    p_target_frame_name_.c_str(), p_source_frame_name_.c_str());

  bool transform_successful = false;

  while (!transform_successful) {
    transform_successful = tf_buffer_.canTransform(
      p_target_frame_name_, p_source_frame_name_,
      tf2::TimePointZero, tf2::durationFromSec(1.0));

    if (transform_successful) {
      break;
    }

    rclcpp::Time now = rclcpp::Node::now();

    if ((now - start).seconds() > 20.0) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "No transform between frams %s and %s available after %f seconds of waiting. This warning only prints once.",
        p_target_frame_name_.c_str(), p_source_frame_name_.c_str(), (now - start).seconds());
    }

    if (!rclcpp::ok()) {
      return;
    }

    rclcpp::WallRate(1.0).sleep();
  }

  rclcpp::Time end = rclcpp::Node::now();
  RCLCPP_INFO(
    get_logger(), "Finished waiting for tf, waited %f seconds", (end - start).seconds());
}

void TrajectoryServer::addCurrentTfPoseToTrajectory()
{
  pose_source_.header.stamp = rclcpp::Time(0);

  geometry_msgs::msg::PoseStamped pose_out;
  tf_buffer_.transform(pose_source_, pose_out, p_target_frame_name_);

  if (trajectory_.poses.size() != 0) {
    //Only add pose to trajectory if it's not already stored
    if (pose_out.header.stamp != trajectory_.poses.back().header.stamp) {
      trajectory_.poses.push_back(pose_out);
    }
  } else {
    trajectory_.poses.push_back(pose_out);
  }
  trajectory_.header.stamp = pose_out.header.stamp;
}

void TrajectoryServer::trajectoryUpdateTimerCallback()
{
  try {
    addCurrentTfPoseToTrajectory();
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(
      get_logger(), "Trajectory Server: Transform from %s to %s failed: %s \n",
      p_target_frame_name_.c_str(), pose_source_.header.frame_id.c_str(), e.what());
  }
}

void TrajectoryServer::publishTrajectoryTimerCallback()
{
  trajectory_pub_->publish(trajectory_);
}

}
