#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <utility>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <qpOASES.hpp>

#include <controller_interface/controller_interface_base.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <pinocchio/algorithm/cholesky.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <crisp_controllers/whole_body_controller.hpp>
#include <crisp_controllers/utils/fiters.hpp>
#include <crisp_controllers/utils/pseudo_inverse.hpp>
#include <crisp_controllers/utils/torque_rate_saturation.hpp>

namespace crisp_controllers {

controller_interface::InterfaceConfiguration
WholeBodyController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // for (const auto & joint_name : params_.joints) {
  //   config.names.push_back(joint_name + "/position");
  // }
  // for (const auto & joint_name : params_.joints) {
  //   config.names.push_back(joint_name + "/velocity");
  // }
  for (const auto & joint_name : params_.joints) {
    config.names.push_back(joint_name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
WholeBodyController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : params_.joints) {
    config.names.push_back(joint_name + "/position");
  }
  for (const auto & joint_name : params_.joints) {
    config.names.push_back(joint_name + "/velocity");
  }
  return config;
}

controller_interface::return_type WholeBodyController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // Read commands and update all rigid-body quantities before assembling the QP.
  
  // Read joint states from hardware interfaces
  if (!updateCurrentState()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Joint state contains NaN/Inf; holding the previous torque command.");
    holdPreviousCommand();
    return controller_interface::return_type::OK;
  }

  // Update the Cartesian target
  updatePoseTargets();

  // Update robot dynamics model
  if (!updateModelAndTasks()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Pinocchio produced NaN/Inf; holding the previous torque command.");
    holdPreviousCommand();
    return controller_interface::return_type::OK;
  }

  // Form Cartesian and posture acceleration references, then solve for torque.
  computeTaskCommands();
  computePostureReference();
  Eigen::VectorXd optimized_torque;
  if (!solveOptimization(optimized_torque)) {
    RCLCPP_INFO(get_node()->get_logger(), "Cannot solve");
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Operational-space QP failed; holding the previous torque command.");
    holdPreviousCommand();
    return controller_interface::return_type::OK;
  }

  // Apply the same output protections used by the existing Cartesian controller.
  Eigen::VectorXd commanded_torque = optimized_torque;
  if (params_.max_delta_tau > 0.0) {
    commanded_torque = saturateTorqueRate(commanded_torque, previous_torque_, params_.max_delta_tau);
  }
  commanded_torque = exponential_moving_average(previous_torque_, commanded_torque, params_.filter.output_torque);
  commanded_torque = commanded_torque.cwiseMin(torque_max_).cwiseMax(torque_min_);

  // Write commands to hardware interfaces 
  writeTorqueCommand(commanded_torque);
  previous_torque_ = commanded_torque;

  if (params_.log.enabled) {
    RCLCPP_INFO_STREAM_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "optimization torque: " << commanded_torque.transpose()
                              << ", first task error: " << tasks_.front().error.transpose());
  }
  return controller_interface::return_type::OK;
}

CallbackReturn WholeBodyController::on_init() {
  try {
    params_listener_ =
      std::make_shared<whole_body_controller::ParamListener>(get_node());
    params_ = params_listener_->get_params();
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter initialization failed: %s", exception.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn WholeBodyController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/) {
  params_ = params_listener_->get_params();
  if (params_.joints.empty() || params_.end_effector_frames.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Both 'joints' and 'end_effector_frames' must be non-empty.");
    return CallbackReturn::ERROR;
  }

  // Load URDF model from robot_description.
  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  if (!parameters_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_state_publisher parameter service is unavailable.");
    return CallbackReturn::ERROR;
  }
  const auto future = parameters_client->get_parameters({"robot_description"});
  const auto result = future.get();
  if (result.empty() || result.front().get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      result.front().as_string().empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to obtain a non-empty robot_description.");
    return CallbackReturn::ERROR;
  }

  // Build topology first, then validate dimension-dependent parameters and inputs.
  // Config robot pinocchio model from robot_description urdf
  // Config necessary parameters
  // Config target cartesian poses and joint pose subscription topics  
  if (!configureModel(result.front().as_string()) || !configureParameters() ||
      !configureSubscriptions()) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured whole-body controller for %zu joints and %zu end-effectors.",
    params_.joints.size(), tasks_.size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn WholeBodyController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!updateCurrentState(true) || !updateModelAndTasks()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot activate with an invalid robot state.");
    return CallbackReturn::ERROR;
  }

  // Start from a Cartesian and joint hold to avoid a command discontinuity.
  q_target_ = q_;
  dq_target_.setZero();
  previous_torque_.setZero();
  joint_target_buffer_.writeFromNonRT(nullptr);
  for (auto & task : tasks_) {
    task.target_pose = task.current_pose;
    task.desired_pose = task.current_pose;
    task.target_buffer->writeFromNonRT(nullptr);
  }

  RCLCPP_INFO(get_node()->get_logger(), "Whole-body controller activated in hold mode.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn WholeBodyController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

bool WholeBodyController::configureModel(const std::string & robot_description) {
  pinocchio::Model raw_model;
  try {
    pinocchio::urdf::buildModelFromXML(robot_description, raw_model);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "URDF parsing failed: %s", exception.what());
    return false;
  }

  // Validate requested joints and lock every other movable joint.
  std::unordered_set<std::string> requested_joints(
    params_.joints.begin(), params_.joints.end());
  if (requested_joints.size() != params_.joints.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "The 'joints' parameter contains duplicates.");
    return false;
  }
  for (const auto & joint_name : params_.joints) {
    if (!raw_model.existJointName(joint_name)) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Joint '%s' is absent from robot_description.", joint_name.c_str());
      return false;
    }
  }
  std::vector<pinocchio::JointIndex> joints_to_lock;
  for (pinocchio::JointIndex joint_id = 1; joint_id < raw_model.joints.size(); ++joint_id) {
    if (requested_joints.count(raw_model.names[joint_id]) == 0U) {
      joints_to_lock.push_back(joint_id);
      RCLCPP_INFO(
        get_node()->get_logger(), "Locking unconfigured joint '%s' at its neutral position.",
        raw_model.names[joint_id].c_str());
    }
  }
  model_ = pinocchio::buildReducedModel(
    raw_model, joints_to_lock, pinocchio::neutral(raw_model));
  data_ = pinocchio::Data(model_);

  // This controller maps one scalar hardware interface to each model velocity.
  if (model_.nq != model_.nv || model_.nv != static_cast<int>(params_.joints.size())) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Only scalar nq==nv joints are supported (model nq=%d, nv=%d, configured=%zu).",
      model_.nq, model_.nv, params_.joints.size());
    return false;
  }
  joint_ids_.clear();
  joint_velocity_indices_.clear();
  for (const auto & joint_name : params_.joints) {
    const auto joint_id = model_.getJointId(joint_name);
    const auto & joint = model_.joints[joint_id];
    if (joint.nq() != 1 || joint.nv() != 1) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Joint '%s' is not a scalar joint.", joint_name.c_str());
      return false;
    }
    joint_ids_.push_back(joint_id);
    joint_velocity_indices_.push_back(joint.idx_v());
  }

  // Resolve each task frame against the reduced model and allocate its Jacobians.
  std::unordered_set<std::string> requested_frames;
  tasks_.clear();
  tasks_.reserve(params_.end_effector_frames.size());
  for (const auto & frame_name : params_.end_effector_frames) {
    if (!requested_frames.insert(frame_name).second || !model_.existFrame(frame_name)) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "End-effector frame '%s' is missing or duplicated.",
        frame_name.c_str());
      return false;
    }
    tasks_.emplace_back();
    auto & task = tasks_.back();
    task.frame_name = frame_name;
    task.frame_id = model_.getFrameId(frame_name);
    task.jacobian = Eigen::MatrixXd::Zero(6, model_.nv);
    task.jacobian_dot = Eigen::MatrixXd::Zero(6, model_.nv);
    task.target_buffer = std::make_unique<realtime_tools::RealtimeBuffer<
      std::shared_ptr<geometry_msgs::msg::PoseStamped>>>();
  }
  return true;
}

bool WholeBodyController::configureParameters() {
  const int joint_count = model_.nv;
  const int task_dimension = 6 * static_cast<int>(tasks_.size());

  // Expand scalar/shared gains into vectors in Pinocchio velocity order.
  Eigen::VectorXd armature;
  if (!expandJointParameter(params_.posture.kp, "posture.kp", posture_kp_) ||
      !expandJointParameter(params_.posture.kd, "posture.kd", posture_kd_) ||
      !expandJointParameter(params_.dynamics.armature, "dynamics.armature", armature) ||
      !expandJointParameter(params_.dynamics.joint_damping, "dynamics.joint_damping", joint_damping_) ||
      !expandTaskParameter(params_.task.kp, "task.kp", task_kp_) ||
      !expandTaskParameter(params_.task.kd, "task.kd", task_kd_)) {
    return false;
  }
  model_.armature = armature;
  error_clip_ = Eigen::VectorXd::Zero(task_dimension);
  for (std::size_t task_index = 0; task_index < tasks_.size(); ++task_index) {
    error_clip_.segment<6>(6 * static_cast<int>(task_index)) =
      Eigen::Map<const Vector6d>(params_.task.error_clip.data());
  }

  // Use explicit symmetric torque limits when supplied, otherwise trust the URDF.
  Eigen::VectorXd torque_limits;
  if (params_.torque_limits.empty()) {
    torque_limits = model_.effortLimit;
  } else if (!expandJointParameter(params_.torque_limits, "torque_limits", torque_limits)) {
    return false;
  }
  if (torque_limits.size() != joint_count || !torque_limits.allFinite() ||
      (torque_limits.array() <= 0.0).any()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Torque limits must be finite and positive; set 'torque_limits' if the URDF has none.");
    return false;
  }
  torque_max_ = torque_limits;
  torque_min_ = -torque_limits;

  // Preallocate all controller state and stacked task workspaces.
  q_ = Eigen::VectorXd::Zero(joint_count);
  dq_ = Eigen::VectorXd::Zero(joint_count);
  q_target_ = Eigen::VectorXd::Zero(joint_count);
  dq_target_ = Eigen::VectorXd::Zero(joint_count);
  qddot_reference_ = Eigen::VectorXd::Zero(joint_count);
  nonlinear_effects_ = Eigen::VectorXd::Zero(joint_count);
  previous_torque_ = Eigen::VectorXd::Zero(joint_count);
  mass_matrix_ = Eigen::MatrixXd::Zero(joint_count, joint_count);
  mass_matrix_inverse_ = Eigen::MatrixXd::Zero(joint_count, joint_count);
  torque_nullspace_projection_ = Eigen::MatrixXd::Identity(joint_count, joint_count);
  stacked_jacobian_ = Eigen::MatrixXd::Zero(task_dimension, joint_count);
  stacked_jdot_qdot_ = Eigen::VectorXd::Zero(task_dimension);
  stacked_xddot_command_ = Eigen::VectorXd::Zero(task_dimension);
  return true;
}

bool WholeBodyController::configureSubscriptions() {
  // Verify the ordered suffix used for each private task-space target topic.
  const auto & pose_topic_suffixes = params_.topics.target_pose;
  if (pose_topic_suffixes.size() != tasks_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "topics.target_pose must match end_effector_frames.");
    return false;
  }
  const std::unordered_set<std::string> unique_suffixes(
    pose_topic_suffixes.begin(), pose_topic_suffixes.end());
  if (unique_suffixes.size() != pose_topic_suffixes.size() ||
      unique_suffixes.count("") != 0U) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "topics.target_pose must contain unique non-empty suffixes.");
    return false;
  }

  // Give every task an independent realtime buffer and PoseStamped subscription.
  for (std::size_t task_index = 0; task_index < tasks_.size(); ++task_index) {
    auto & task = tasks_[task_index];
    task.topic_name = pose_topic_suffixes[task_index];
    auto * target_buffer = task.target_buffer.get();
    task.subscription = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      task.topic_name, rclcpp::QoS(1),
      [target_buffer](const std::shared_ptr<geometry_msgs::msg::PoseStamped> message) {
        target_buffer->writeFromNonRT(message);
      });
    RCLCPP_INFO(
      get_node()->get_logger(), "Pose target for '%s': %s", task.frame_name.c_str(),
      task.topic_name.c_str());
  }

  // Subscribe to the posture target in the same controller-private topic hierarchy.
  if (params_.topics.target_joint.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "topics.target_joint must be non-empty.");
    return false;
  }
  const std::string joint_target_topic = params_.topics.target_joint;
  joint_target_subscription_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    joint_target_topic, rclcpp::QoS(1),
    [this](const std::shared_ptr<sensor_msgs::msg::JointState> message) {
      joint_target_buffer_.writeFromNonRT(message);
    });
  RCLCPP_INFO(get_node()->get_logger(), "Joint target: %s", joint_target_topic.c_str());
  return true;
}

bool WholeBodyController::updateCurrentState(bool /*initialize*/) {
  const std::size_t joint_count = params_.joints.size();
  for (std::size_t parameter_index = 0; parameter_index < joint_count; ++parameter_index) {
    const int model_index = joint_velocity_indices_[parameter_index];
#if ROS2_VERSION_ABOVE_HUMBLE
    const double position = state_interfaces_[parameter_index].get_optional().value_or(
      q_[model_index]);
    const double velocity = state_interfaces_[joint_count + parameter_index].get_optional().value_or(
      dq_[model_index]);
#else
    const double position = state_interfaces_[parameter_index].get_value();
    const double velocity = state_interfaces_[joint_count + parameter_index].get_value();
#endif
    if (!std::isfinite(position) || !std::isfinite(velocity)) {
      return false;
    }
    q_[model_index] = position;
    dq_[model_index] = velocity;
  }
  return true;
}

void WholeBodyController::updatePoseTargets() {
  for (auto & task : tasks_) {
    const auto message = *task.target_buffer->readFromRT();
    if (!message) {
      continue;
    }
    if (!params_.base_frame.empty() && !message->header.frame_id.empty() &&
        message->header.frame_id != params_.base_frame) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Ignoring target for '%s': frame '%s' is not configured base frame '%s'.",
        task.frame_name.c_str(), message->header.frame_id.c_str(), params_.base_frame.c_str());
      continue;
    }

    const auto & pose = message->pose;
    const Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond orientation(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    if (!position.allFinite() || !orientation.coeffs().allFinite() ||
        orientation.norm() < 1.0e-9) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Ignoring invalid pose target for '%s'.", task.frame_name.c_str());
      continue;
    }
    orientation.normalize();
    task.target_pose = pinocchio::SE3(orientation.toRotationMatrix(), position);
  }
}

bool WholeBodyController::updateModelAndTasks() {
  // Update kinematics, Jacobians, and Jacobian time variation once for all tasks.
  pinocchio::forwardKinematics(model_, data_, q_, dq_);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::computeJointJacobians(model_, data_, q_);
  pinocchio::computeJointJacobiansTimeVariation(model_, data_, q_, dq_);

  // Factor the CRBA mass matrix and compute its inverse as in the reference FSM.
  pinocchio::crba(model_, data_, q_);
  data_.M.triangularView<Eigen::StrictlyLower>() =
    data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::cholesky::decompose(model_, data_);
  pinocchio::cholesky::computeMinv(model_, data_);
  data_.Minv.triangularView<Eigen::StrictlyLower>() =
    data_.Minv.transpose().triangularView<Eigen::StrictlyLower>();
  mass_matrix_ = data_.M;
  mass_matrix_inverse_ = data_.Minv;

  // Assemble independently selectable rigid-body compensation terms.
  nonlinear_effects_.setZero();
  if (params_.dynamics.use_coriolis) {
    pinocchio::computeCoriolisMatrix(model_, data_, q_, dq_);
    nonlinear_effects_.noalias() += data_.C * dq_;
  }
  if (params_.dynamics.use_gravity) {
    nonlinear_effects_ += pinocchio::computeGeneralizedGravity(model_, data_, q_);
  }
  nonlinear_effects_ += joint_damping_.cwiseProduct(dq_);

  // Read each frame pose and its world-aligned spatial derivatives.
  for (auto & task : tasks_) {
    task.current_pose = data_.oMf[task.frame_id];
    task.jacobian.setZero();
    task.jacobian_dot.setZero();
    pinocchio::getFrameJacobian(
      model_, data_, task.frame_id, pinocchio::LOCAL_WORLD_ALIGNED, task.jacobian);
    pinocchio::getFrameJacobianTimeVariation(
      model_, data_, task.frame_id, pinocchio::LOCAL_WORLD_ALIGNED, task.jacobian_dot);
    if (!task.current_pose.translation().allFinite() ||
        !task.current_pose.rotation().allFinite() || !task.jacobian.allFinite() ||
        !task.jacobian_dot.allFinite()) {
      return false;
    }
  }
  return mass_matrix_.allFinite() && mass_matrix_inverse_.allFinite() &&
         nonlinear_effects_.allFinite();
}

void WholeBodyController::computeTaskCommands() {
  const double previous_weight = params_.filter.target_pose;
  stacked_jacobian_.setZero();
  stacked_jdot_qdot_.setZero();
  stacked_xddot_command_.setZero();

  for (std::size_t task_index = 0; task_index < tasks_.size(); ++task_index) {
    auto & task = tasks_[task_index];
    const int row = 6 * static_cast<int>(task_index);

    // Smooth translation and rotation independently to avoid screw interpolation.
    task.desired_pose.translation() = exponential_moving_average(
      task.desired_pose.translation(), task.target_pose.translation(), previous_weight);
    const Eigen::Quaterniond target_orientation(task.target_pose.rotation());
    const Eigen::Quaterniond previous_orientation(task.desired_pose.rotation());
    const Eigen::Quaterniond desired_orientation =
      target_orientation.slerp(previous_weight, previous_orientation).normalized();
    task.desired_pose.rotation() = desired_orientation.toRotationMatrix();

    // Build the world-aligned PD acceleration target for this frame.
    task.error.head<3>() = 
      task.desired_pose.translation() - task.current_pose.translation();
    task.error.tail<3>() = pinocchio::log3(
      task.desired_pose.rotation() * task.current_pose.rotation().transpose());
    task.error = task.error.cwiseMin(error_clip_.segment<6>(row)).cwiseMax(
      -error_clip_.segment<6>(row));
    const Vector6d task_velocity = task.jacobian * dq_;
    task.xddot_command = task_kp_.segment<6>(row).cwiseProduct(task.error) -
      task_kd_.segment<6>(row).cwiseProduct(task_velocity);

    stacked_jacobian_.block(row, 0, 6, model_.nv) = task.jacobian;
    stacked_jdot_qdot_.segment<6>(row) = task.jacobian_dot * dq_;
    stacked_xddot_command_.segment<6>(row) = task.xddot_command;

    // Zero-gain axes are removed from the QP instead of becoming passive constraints.
    for (int axis = 0; axis < 6; ++axis) {
      if (std::abs(task_kp_[row + axis]) <= 1.0e-9 &&
          std::abs(task_kd_[row + axis]) <= 1.0e-9) {
        stacked_jacobian_.row(row + axis).setZero();
        stacked_jdot_qdot_[row + axis] = 0.0;
        stacked_xddot_command_[row + axis] = 0.0;
      }
    }
  }
}

void WholeBodyController::computePostureReference() {
  const auto message = *joint_target_buffer_.readFromRT();
  if (message) {
    // Named commands may be sparse; unnamed commands follow configured joint order.
    if (!message->name.empty()) {
      for (std::size_t message_index = 0; message_index < message->name.size(); ++message_index) {
        const auto configured = std::find(
          params_.joints.begin(), params_.joints.end(), message->name[message_index]);
        if (configured == params_.joints.end()) {
          continue;
        }
        const auto parameter_index =
          static_cast<std::size_t>(std::distance(params_.joints.begin(), configured));
        const int model_index = joint_velocity_indices_[parameter_index];
        if (message_index < message->position.size() &&
            std::isfinite(message->position[message_index])) {
          q_target_[model_index] = message->position[message_index];
        }
        if (message_index < message->velocity.size() &&
            std::isfinite(message->velocity[message_index])) {
          dq_target_[model_index] = message->velocity[message_index];
        }
      }
    } else {
      const auto count = std::min(message->position.size(), params_.joints.size());
      for (std::size_t parameter_index = 0; parameter_index < count; ++parameter_index) {
        if (std::isfinite(message->position[parameter_index])) {
          q_target_[joint_velocity_indices_[parameter_index]] = message->position[parameter_index];
        }
      }
      const auto velocity_count = std::min(message->velocity.size(), params_.joints.size());
      for (std::size_t parameter_index = 0; parameter_index < velocity_count; ++parameter_index) {
        if (std::isfinite(message->velocity[parameter_index])) {
          dq_target_[joint_velocity_indices_[parameter_index]] = message->velocity[parameter_index];
        }
      }
    }
  }
  qddot_reference_ = posture_kp_.cwiseProduct(q_target_ - q_) +
    posture_kd_.cwiseProduct(dq_target_ - dq_);
}

bool WholeBodyController::solveOptimization(Eigen::VectorXd & torque_solution) {
  const int decision_dimension = model_.nv;

  // Select how the secondary acceleration objective interacts with Cartesian tasks.
  const Eigen::MatrixXd identity =
    Eigen::MatrixXd::Identity(decision_dimension, decision_dimension);
  const Eigen::MatrixXd jacobian_minv = stacked_jacobian_ * mass_matrix_inverse_;
  if (params_.nullspace.projector_type == "dynamic") {
    const Eigen::MatrixXd lambda_inverse =
      jacobian_minv * stacked_jacobian_.transpose();
    const Eigen::MatrixXd lambda =
      pseudo_inverse(lambda_inverse, params_.nullspace.regularization);
    torque_nullspace_projection_ =
      identity - stacked_jacobian_.transpose() * lambda * jacobian_minv;
  } else if (params_.nullspace.projector_type == "kinematic") {
    const Eigen::MatrixXd jacobian_pseudoinverse =
      pseudo_inverse(stacked_jacobian_, params_.nullspace.regularization);
    torque_nullspace_projection_ = identity - jacobian_pseudoinverse * stacked_jacobian_;
  } else if (params_.nullspace.projector_type == "none") {
    torque_nullspace_projection_ = identity;
  } else {
    RCLCPP_ERROR_STREAM_ONCE(
      get_node()->get_logger(),
      "Unknown nullspace projector type: " << params_.nullspace.projector_type);
    return false;
  }

  // Assemble the reference FSM's least-squares costs in generalized motor torque.
  const Eigen::MatrixXd task_cost_matrix = jacobian_minv; // J * M_inv
  const Eigen::VectorXd task_cost_target = stacked_xddot_command_ - stacked_jdot_qdot_; // xdd_r - J * qd
  const Eigen::MatrixXd posture_cost_matrix = mass_matrix_inverse_ * torque_nullspace_projection_; // M_inv * N
  const Eigen::VectorXd posture_cost_target = posture_cost_matrix * mass_matrix_ * qddot_reference_; // M_inv * N * M * qdd_r

  Eigen::MatrixXd hessian =
    params_.weights.task * task_cost_matrix.transpose() * task_cost_matrix +
    params_.weights.qddot * posture_cost_matrix.transpose() * posture_cost_matrix +
    params_.weights.regularization_qddot * mass_matrix_inverse_.transpose() * mass_matrix_inverse_;
  
  hessian.diagonal().array() += params_.weights.regularization; 
  hessian = 0.5 * (hessian + hessian.transpose());
  
  const Eigen::VectorXd gradient =
    - params_.weights.task * task_cost_matrix.transpose() * task_cost_target 
    - params_.weights.qddot * posture_cost_matrix.transpose() * posture_cost_target;

  const Eigen::VectorXd lower_bound = torque_min_ - nonlinear_effects_;
  const Eigen::VectorXd upper_bound = torque_max_ - nonlinear_effects_;
  using RowMajorMatrix =
    Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  const RowMajorMatrix qp_hessian = hessian.cast<qpOASES::real_t>();
  const Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1> qp_gradient =
    gradient.cast<qpOASES::real_t>();
  const Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1> qp_lower_bound =
    lower_bound.cast<qpOASES::real_t>();
  const Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1> qp_upper_bound =
    upper_bound.cast<qpOASES::real_t>();

  // Solve the torque-box-constrained convex QP.
  qpOASES::QProblemB problem(decision_dimension);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  options.enableRegularisation = qpOASES::BT_TRUE;
  problem.setOptions(options);
  int working_set_recalculations = params_.qp.max_working_set_recalculations;
  const auto status = problem.init(
    qp_hessian.data(), qp_gradient.data(), qp_lower_bound.data(), qp_upper_bound.data(),
    working_set_recalculations);
  if (status != qpOASES::SUCCESSFUL_RETURN || problem.isSolved() != qpOASES::BT_TRUE) {
    return false;
  }

  Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1> motor_torque(decision_dimension);
  if (problem.getPrimalSolution(motor_torque.data()) != qpOASES::SUCCESSFUL_RETURN) {
    return false;
  }
  torque_solution = motor_torque.cast<double>() + nonlinear_effects_;
  return torque_solution.allFinite();
}

void WholeBodyController::writeTorqueCommand(const Eigen::VectorXd & torque) {
  if (params_.stop_commands) {
    return;
  }
  const std::size_t joint_count = params_.joints.size();
  for (std::size_t parameter_index = 0; parameter_index < joint_count; ++parameter_index) {
    const int model_index = joint_velocity_indices_[parameter_index];
    // const double position_command = q_[model_index];
    // const double velocity_command = dq_[model_index];
    const double effort_command = torque[model_index];
#if ROS2_VERSION_ABOVE_HUMBLE
    // (void)command_interfaces_[parameter_index].set_value(position_command);
    // (void)command_interfaces_[joint_count + parameter_index].set_value(velocity_command);
    // (void)command_interfaces_[2 * joint_count + parameter_index].set_value(effort_command);
    (void)command_interfaces_[parameter_index].set_value(effort_command);
#else
    // command_interfaces_[parameter_index].set_value(position_command);
    // command_interfaces_[joint_count + parameter_index].set_value(velocity_command);
    // command_interfaces_[joint_count * 2 + parameter_index].set_value(effort_command);
    command_interfaces_[parameter_index].set_value(effort_command);
#endif
  }
}

void WholeBodyController::holdPreviousCommand() {
  writeTorqueCommand(previous_torque_);
}

bool WholeBodyController::expandJointParameter(
  const std::vector<double> & input, const std::string & name, Eigen::VectorXd & output,
  bool allow_empty) const {
  if (input.empty() && allow_empty) {
    output.resize(0);
    return true;
  }
  if (input.size() != 1U && input.size() != params_.joints.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Parameter '%s' must contain one or %zu values (got %zu).",
      name.c_str(), params_.joints.size(), input.size());
    return false;
  }
  output = Eigen::VectorXd::Zero(model_.nv);
  for (std::size_t parameter_index = 0; parameter_index < params_.joints.size(); ++parameter_index) {
    const double value = input.size() == 1U ? input.front() : input[parameter_index];
    if (!std::isfinite(value) || value < 0.0) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Parameter '%s' must contain finite non-negative values.",
        name.c_str());
      return false;
    }
    output[joint_velocity_indices_[parameter_index]] = value;
  }
  return true;
}

bool WholeBodyController::expandTaskParameter(
  const std::vector<double> & input, const std::string & name, Eigen::VectorXd & output) const {
  const std::size_t task_dimension = 6U * tasks_.size();
  if (input.size() != 6U && input.size() != task_dimension) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Parameter '%s' must contain 6 or %zu values (got %zu).",
      name.c_str(), task_dimension, input.size());
    return false;
  }
  output = Eigen::VectorXd::Zero(static_cast<int>(task_dimension));
  for (std::size_t index = 0; index < task_dimension; ++index) {
    const double value = input.size() == 6U ? input[index % 6U] : input[index];
    if (!std::isfinite(value) || value < 0.0) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Parameter '%s' must contain finite non-negative values.",
        name.c_str());
      return false;
    }
    output[static_cast<int>(index)] = value;
  }
  return true;
}

}  // namespace crisp_controllers

PLUGINLIB_EXPORT_CLASS(
  crisp_controllers::WholeBodyController, controller_interface::ControllerInterface)
