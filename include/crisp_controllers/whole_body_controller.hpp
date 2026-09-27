#pragma once

/**
 * @file whole_body_controller.hpp
 * @brief Acceleration-level QP controller for multiple Cartesian end effectors.
 */

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>  // NOLINT(build/include_order)
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <crisp_controllers/utils/ros2_version.hpp>

#if ROS2_VERSION_ABOVE_HUMBLE
#include <crisp_controllers/whole_body_controller_parameters.hpp>
#else
#include <whole_body_controller_parameters.hpp>
#endif

#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace crisp_controllers {

/**
 * @brief Multi-end-effector operational controller solved as a torque-bounded QP.
 *
 * The decision variable is generalized motor torque. Cartesian acceleration
 * tracking is the primary cost and a selectable projected joint-posture
 * acceleration is the secondary cost.
 */
class WholeBodyController : public controller_interface::ControllerInterface {
public:
  WholeBodyController() = default;
  ~WholeBodyController() override = default;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type
  update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

private:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using JointStateRealtimePublisher =
    realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>;

  struct TorqueDiagnostic {
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    std::shared_ptr<JointStateRealtimePublisher> realtime_publisher;
    sensor_msgs::msg::JointState message;
  };

  struct EndEffectorTask {
    std::string frame_name;
    std::string topic_name;
    std::string velocity_topic_name;
    pinocchio::FrameIndex frame_id{0};
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription;
    std::unique_ptr<realtime_tools::RealtimeBuffer<
      std::shared_ptr<geometry_msgs::msg::PoseStamped>>> target_buffer;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription;
    std::unique_ptr<realtime_tools::RealtimeBuffer<
      std::shared_ptr<geometry_msgs::msg::TwistStamped>>> velocity_target_buffer;

    pinocchio::SE3 current_pose{pinocchio::SE3::Identity()};
    pinocchio::SE3 target_pose{pinocchio::SE3::Identity()};
    pinocchio::SE3 desired_pose{pinocchio::SE3::Identity()};
    Vector6d target_velocity{Vector6d::Zero()};
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd jacobian_dot;
    Vector6d error{Vector6d::Zero()};
    Vector6d xddot_command{Vector6d::Zero()};
  };

  bool configureModel(const std::string & robot_description);
  bool configureParameters();
  bool configureSubscriptions();
  bool configureTorqueDiagnostics();
  bool updateCurrentState(bool initialize = false);
  void updatePoseTargets();
  bool updateModelAndTasks();
  void computeTaskCommands();
  void computePostureReference();
  bool solveOptimization(Eigen::VectorXd & torque_solution);
  void publishTorqueDecomposition(
    const rclcpp::Time & time, const rclcpp::Duration & period,
    const Eigen::VectorXd & command_torque);
  void publishTorqueDiagnostic(
    TorqueDiagnostic & diagnostic, const rclcpp::Time & time,
    const Eigen::VectorXd & torque);
  void writeTorqueCommand(const Eigen::VectorXd & torque);
  void holdPreviousCommand();

  bool expandJointParameter(
    const std::vector<double> & input, const std::string & name, Eigen::VectorXd & output,
    bool allow_empty = false) const;
  bool expandTaskParameter(
    const std::vector<double> & input, const std::string & name, Eigen::VectorXd & output) const;

  std::shared_ptr<whole_body_controller::ParamListener> params_listener_;
  whole_body_controller::Params params_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_target_subscription_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>>
    joint_target_buffer_;
  std::vector<EndEffectorTask> tasks_;

  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<pinocchio::JointIndex> joint_ids_;
  std::vector<int> joint_velocity_indices_;

  Eigen::VectorXd q_;
  Eigen::VectorXd dq_;
  Eigen::VectorXd q_target_;
  Eigen::VectorXd dq_target_;
  Eigen::VectorXd qddot_reference_;
  Eigen::VectorXd posture_kp_;
  Eigen::VectorXd posture_kd_;
  Eigen::VectorXd feedback_kp_;
  Eigen::VectorXd feedback_kd_;
  Eigen::VectorXd joint_damping_;

  Eigen::VectorXd task_kp_;
  Eigen::VectorXd task_kd_;
  Eigen::VectorXd error_clip_;
  Eigen::MatrixXd stacked_jacobian_;
  Eigen::VectorXd stacked_jdot_qdot_;
  Eigen::VectorXd stacked_xddot_command_;

  Eigen::MatrixXd mass_matrix_;
  Eigen::MatrixXd mass_matrix_inverse_;
  Eigen::MatrixXd torque_nullspace_projection_;
  Eigen::VectorXd motion_torque_;
  Eigen::VectorXd nonlinear_effects_;
  Eigen::VectorXd feedforward_torque_;
  Eigen::VectorXd feedback_torque_;
  Eigen::VectorXd torque_min_;
  Eigen::VectorXd torque_max_;
  Eigen::VectorXd previous_torque_;

  TorqueDiagnostic motion_diagnostic_;
  TorqueDiagnostic nonlinear_diagnostic_;
  TorqueDiagnostic feedforward_diagnostic_;
  TorqueDiagnostic feedback_diagnostic_;
  TorqueDiagnostic command_diagnostic_;
  rclcpp::Duration decomposition_elapsed_{0, 0};
  rclcpp::Duration decomposition_interval_{0, 0};
};

}  // namespace crisp_controllers
