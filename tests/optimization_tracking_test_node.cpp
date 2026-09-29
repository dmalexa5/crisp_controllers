#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace crisp_controllers {

class OptimizationTrackingTestNode : public rclcpp::Node {
public:
  explicit OptimizationTrackingTestNode(const rclcpp::NodeOptions & options)
  : Node("optimization_tracking_test_node", options),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
    base_frame_ = declare_parameter<std::string>("base_frame", "rail_link");
    publish_rate_ = declare_parameter<double>("publish_rate", 100.0);
    startup_delay_ = declare_parameter<double>("startup_delay", 2.0);
    if (publish_rate_ <= 0.0 || startup_delay_ < 0.0) {
      throw std::invalid_argument("publish_rate must be positive and startup_delay non-negative");
    }

    // Load one independently configurable trajectory for each controller task.
    left_ = loadArmMotion(
      "left", "left_fr3_hand_tcp", "whole_body_controller/target_pose/left",
      "whole_body_controller/target_velocity/left", "linear");
    right_ = loadArmMotion(
      "right", "right_fr3_hand_tcp", "whole_body_controller/target_pose/right",
      "whole_body_controller/target_velocity/right", "circular");
    validateArmMotion(left_);
    validateArmMotion(right_);

    left_.publisher = create_publisher<geometry_msgs::msg::PoseStamped>(left_.topic, rclcpp::QoS(1));
    right_.publisher =
      create_publisher<geometry_msgs::msg::PoseStamped>(right_.topic, rclcpp::QoS(1));
    left_.velocity_publisher =
      create_publisher<geometry_msgs::msg::TwistStamped>(left_.velocity_topic, rclcpp::QoS(1));
    right_.velocity_publisher =
      create_publisher<geometry_msgs::msg::TwistStamped>(right_.velocity_topic, rclcpp::QoS(1));

    const auto period = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
    timer_ = rclcpp::create_timer(
      get_node_base_interface(), get_node_timers_interface(), get_clock(), period,
      std::bind(&OptimizationTrackingTestNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(), "Waiting for transforms from '%s' to '%s' and '%s'.", base_frame_.c_str(),
      left_.frame.c_str(), right_.frame.c_str());
  }

private:
  struct ArmMotion {
    std::string name;
    std::string frame;
    std::string topic;
    std::string velocity_topic;
    std::string motion_type;
    double linear_amplitude{0.05};
    double linear_speed{0.5};
    std::array<double, 3> linear_direction{1.0, 0.0, 0.0};
    double circle_radius{0.05};
    double circle_speed{0.5};
    std::array<double, 3> circle_axis_u{1.0, 0.0, 0.0};
    std::array<double, 3> circle_axis_v{0.0, 1.0, 0.0};
    geometry_msgs::msg::Pose home_pose;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher;
  };

  ArmMotion loadArmMotion(
    const std::string & name, const std::string & default_frame,
    const std::string & default_topic, const std::string & default_velocity_topic,
    const std::string & default_motion) {
    ArmMotion motion;
    motion.name = name;
    motion.frame = declare_parameter<std::string>(name + ".frame", default_frame);
    motion.topic = declare_parameter<std::string>(name + ".topic", default_topic);
    motion.velocity_topic =
      declare_parameter<std::string>(name + ".velocity_topic", default_velocity_topic);
    motion.motion_type = declare_parameter<std::string>(name + ".motion_type", default_motion);
    motion.linear_amplitude = declare_parameter<double>(name + ".linear_amplitude", 0.05);
    motion.linear_speed = declare_parameter<double>(name + ".linear_speed", 0.5);
    motion.linear_direction = toArray(
      declare_parameter<std::vector<double>>(
        name + ".linear_direction", std::vector<double>{1.0, 0.0, 0.0}),
      name + ".linear_direction");
    motion.circle_radius = declare_parameter<double>(name + ".circle_radius", 0.05);
    motion.circle_speed = declare_parameter<double>(name + ".circle_speed", 0.5);
    motion.circle_axis_u = toArray(
      declare_parameter<std::vector<double>>(
        name + ".circle_axis_u", std::vector<double>{1.0, 0.0, 0.0}),
      name + ".circle_axis_u");
    motion.circle_axis_v = toArray(
      declare_parameter<std::vector<double>>(
        name + ".circle_axis_v", std::vector<double>{0.0, 1.0, 0.0}),
      name + ".circle_axis_v");
    return motion;
  }

  static std::array<double, 3> toArray(
    const std::vector<double> & values, const std::string & parameter_name) {
    if (values.size() != 3U ||
        !std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);})) {
      throw std::invalid_argument(parameter_name + " must contain three finite values");
    }
    return {values[0], values[1], values[2]};
  }

  static double norm(const std::array<double, 3> & vector) {
    return std::sqrt(
      vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
  }

  static double dot(
    const std::array<double, 3> & lhs, const std::array<double, 3> & rhs) {
    return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
  }

  static void normalize(std::array<double, 3> & vector) {
    const double length = norm(vector);
    if (length <= 1.0e-9) {
      throw std::invalid_argument("trajectory direction/axis must be non-zero");
    }
    for (double & value : vector) {
      value /= length;
    }
  }

  static void validateArmMotion(ArmMotion & motion) {
    if (motion.motion_type != "linear" && motion.motion_type != "circular" &&
        motion.motion_type != "hold") {
      throw std::invalid_argument(
              motion.name + ".motion_type must be linear, circular, or hold");
    }
    if (motion.linear_amplitude < 0.0 || motion.linear_speed < 0.0 ||
        motion.circle_radius < 0.0 || motion.circle_speed < 0.0) {
      throw std::invalid_argument("trajectory amplitudes, radii, and speeds must be non-negative");
    }
    normalize(motion.linear_direction);
    normalize(motion.circle_axis_u);

    // Gram-Schmidt makes the configured circular plane orthonormal.
    const double projection = dot(motion.circle_axis_v, motion.circle_axis_u);
    for (std::size_t index = 0; index < 3U; ++index) {
      motion.circle_axis_v[index] -= projection * motion.circle_axis_u[index];
    }
    normalize(motion.circle_axis_v);
  }

  bool initializeHomePoses() {
    try {
      const auto left_transform =
        tf_buffer_->lookupTransform(base_frame_, left_.frame, tf2::TimePointZero);
      const auto right_transform =
        tf_buffer_->lookupTransform(base_frame_, right_.frame, tf2::TimePointZero);
      left_.home_pose = transformToPose(left_transform);
      right_.home_pose = transformToPose(right_transform);
      trajectory_start_time_ = now();
      initialized_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Tracking initialized. Holding for %.2f s, then left=%s and right=%s.", startup_delay_,
        left_.motion_type.c_str(), right_.motion_type.c_str());
      return true;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for end-effector transforms: %s",
        exception.what());
      return false;
    }
  }

  static geometry_msgs::msg::Pose transformToPose(
    const geometry_msgs::msg::TransformStamped & transform) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation = transform.transform.rotation;
    return pose;
  }

  geometry_msgs::msg::PoseStamped makeTarget(
    const ArmMotion & motion, double motion_time, const rclcpp::Time & stamp) const {
    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = stamp;
    target.header.frame_id = base_frame_;
    target.pose = motion.home_pose;

    if (motion.motion_type == "linear") {
      const double displacement =
        motion.linear_amplitude * std::sin(motion.linear_speed * motion_time);
      target.pose.position.x += displacement * motion.linear_direction[0];
      target.pose.position.y += displacement * motion.linear_direction[1];
      target.pose.position.z += displacement * motion.linear_direction[2];
    } else if (motion.motion_type == "circular") {
      const double angle = motion.circle_speed * motion_time;
      const double displacement_u = motion.circle_radius * (std::cos(angle) - 1.0);
      const double displacement_v = motion.circle_radius * std::sin(angle);
      target.pose.position.x +=
        displacement_u * motion.circle_axis_u[0] + displacement_v * motion.circle_axis_v[0];
      target.pose.position.y +=
        displacement_u * motion.circle_axis_u[1] + displacement_v * motion.circle_axis_v[1];
      target.pose.position.z +=
        displacement_u * motion.circle_axis_u[2] + displacement_v * motion.circle_axis_v[2];
    }
    return target;
  }

  // Analytic time derivative of makeTarget(); zero while holding before the motion starts.
  geometry_msgs::msg::TwistStamped makeVelocityTarget(
    const ArmMotion & motion, double motion_time, bool moving, const rclcpp::Time & stamp) const {
    geometry_msgs::msg::TwistStamped target;
    target.header.stamp = stamp;
    target.header.frame_id = base_frame_;
    if (!moving) {
      return target;
    }

    if (motion.motion_type == "linear") {
      const double speed = motion.linear_amplitude * motion.linear_speed *
        std::cos(motion.linear_speed * motion_time);
      target.twist.linear.x = speed * motion.linear_direction[0];
      target.twist.linear.y = speed * motion.linear_direction[1];
      target.twist.linear.z = speed * motion.linear_direction[2];
    } else if (motion.motion_type == "circular") {
      const double angle = motion.circle_speed * motion_time;
      const double velocity_u = -motion.circle_radius * motion.circle_speed * std::sin(angle);
      const double velocity_v = motion.circle_radius * motion.circle_speed * std::cos(angle);
      target.twist.linear.x =
        velocity_u * motion.circle_axis_u[0] + velocity_v * motion.circle_axis_v[0];
      target.twist.linear.y =
        velocity_u * motion.circle_axis_u[1] + velocity_v * motion.circle_axis_v[1];
      target.twist.linear.z =
        velocity_u * motion.circle_axis_u[2] + velocity_v * motion.circle_axis_v[2];
    }
    return target;
  }

  void onTimer() {
    // Wait for both live poses so the first published targets equal the current poses.
    if (!initialized_ && !initializeHomePoses()) {
      return;
    }

    // Publish a hold during startup, then advance both trajectories on the ROS clock.
    // The controller pairs pose and twist by identical stamps, so both share one timestamp.
    const rclcpp::Time stamp = now();
    const double elapsed = (stamp - trajectory_start_time_).seconds();
    const double motion_time = std::max(0.0, elapsed - startup_delay_);
    const bool moving = elapsed > startup_delay_;
    for (ArmMotion * motion : {&left_, &right_}) {
      motion->publisher->publish(makeTarget(*motion, motion_time, stamp));
      motion->velocity_publisher->publish(
        makeVelocityTarget(*motion, motion_time, moving, stamp));
    }
  }

  std::string base_frame_;
  double publish_rate_{200.0};
  double startup_delay_{2.0};
  ArmMotion left_;
  ArmMotion right_;
  bool initialized_{false};
  rclcpp::Time trajectory_start_time_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace crisp_controllers

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    // Load the installed test trajectory while retaining global ROS arguments.
    const std::string parameter_file =
      ament_index_cpp::get_package_share_directory("crisp_controllers") +
      "/config/optimization_tracking_test.yaml";
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", parameter_file});
    auto node = std::make_shared<crisp_controllers::OptimizationTrackingTestNode>(options);
    RCLCPP_INFO(node->get_logger(), "Loaded tracking parameters from '%s'.", parameter_file.c_str());
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("optimization_tracking_test_node"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
