# Whole-body controller

`crisp_controllers/WholeBodyController` is a torque-bounded, acceleration-level QP controller for one or more Cartesian end-effector tasks. It combines the task objectives with a secondary joint-posture objective and optional rigid-body compensation from Pinocchio.

## Controller and hardware setup

Register the plugin with `controller_manager`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000
    whole_body_controller:
      type: crisp_controllers/WholeBodyController
```

Every entry in `joints` must expose position and velocity state interfaces. The controller currently claims position, velocity, and effort command interfaces and writes the measured position and velocity together with the computed torque. Hardware that implements pure effort control should ignore the position and velocity fields. For an MIT-style actuator, disable its internal position and velocity gains when the QP torque is intended to be the complete command.

The controller reads `robot_description` from `robot_state_publisher`, builds a Pinocchio model, and retains only the movable joints listed in `joints`.

!!! warning "Joints omitted from `joints` are fixed"
    Any movable URDF joint not listed in `joints` is locked at its Pinocchio neutral position by `buildReducedModel`. Its attached link mass and inertia remain in the reduced model, but the joint does not appear in the state vector, mass matrix, QP, or torque output.

    For example, omitting a rail joint makes the controller assume that the rail is rigidly fixed at its neutral position, normally zero. This is valid only when the physical or simulated rail is held at that same position. A free, moving, or nonzero rail requires a model that retains its state; the current controller does not support a moving unactuated joint separately from `joints`.

## Topics and startup behavior

Topic settings are suffixes in the controller's private namespace:

```yaml
whole_body_controller:
  ros__parameters:
    end_effector_frames: [left_tcp, right_tcp]
    topics:
      target_pose: [~/target_pose/left, ~/target_pose/right]
      target_velocity: [~/target_velocity/left, ~/target_velocity/right]
      target_joint: ~/target_joint
```

For a controller named `whole_body_controller`, these settings create:

```text
/whole_body_controller/target_pose/left   geometry_msgs/msg/PoseStamped
/whole_body_controller/target_pose/right  geometry_msgs/msg/PoseStamped
/whole_body_controller/target_velocity/left   geometry_msgs/msg/TwistStamped
/whole_body_controller/target_velocity/right  geometry_msgs/msg/TwistStamped
/whole_body_controller/target_joint       sensor_msgs/msg/JointState
```

`topics.target_pose` and `end_effector_frames` are ordered lists with the same length.
`topics.target_velocity` is optional: use an empty list to disable all velocity subscriptions, or
provide one entry per end effector and leave individual entries empty as needed. A twist is used
only when its timestamp matches the corresponding pose. If the velocity topic is disabled, no
twist has arrived, or the latest twist does not match the pose timestamp, the target velocity is
zero. A target may use an empty `header.frame_id`, or it must equal `base_frame` when
`base_frame` is configured. The controller does not transform targets between frames. The `~/`
prefix makes these topics private to the controller node; omitting it resolves them in the node
namespace instead. Its Cartesian acceleration reference is
`xddot_d = Kp * pose_error + Kd * (xdot_d - J(q) * qdot)`.

On activation, Cartesian targets hold the measured end-effector poses. The joint target defaults
to `posture.nominal`, with zero target joint velocity, until a `target_joint` message is available.
Named `JointState` commands may update a subset of controlled joints; unspecified positions retain
their nominal values and unspecified velocities remain zero. Unnamed commands follow `joints` order.

## Dynamics

Pinocchio provides the controlled-joint dynamics

$$
\boldsymbol{\tau} = \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}}
  + \mathbf{C}(\mathbf{q},\dot{\mathbf{q}})\dot{\mathbf{q}}
  + \mathbf{g}(\mathbf{q})
  + \mathbf{D}\dot{\mathbf{q}}.
$$

The configured armature is added to the rigid-body mass matrix diagonal:

$$
\mathbf{M}(\mathbf{q}) = \mathbf{M}_{\text{URDF}}(\mathbf{q})
  + \operatorname{diag}(\mathbf{a}).
$$

The compensation vector used by the QP is

$$
\mathbf{h} =
  s_C\,\mathbf{C}(\mathbf{q},\dot{\mathbf{q}})\dot{\mathbf{q}}
  + s_g\,\mathbf{g}(\mathbf{q})
  + \mathbf{D}\dot{\mathbf{q}},
$$

where `dynamics.use_coriolis` controls $s_C$ and `dynamics.use_gravity` controls $s_g$. Joint damping is always included; set `dynamics.joint_damping: [0.0]` to disable it.

| Desired behavior | `use_coriolis` | `use_gravity` | Typical use |
| --- | --- | --- | --- |
| Full model compensation | `true` | `true` | Direct torque control when hardware adds neither term |
| Gravity only | `false` | `true` | Slow motion or hardware that already handles velocity effects |
| Coriolis only | `true` | `false` | Hardware that already compensates gravity |
| No rigid-body bias compensation | `false` | `false` | Hardware that supplies both terms, or controlled comparison tests |

Do not enable a term both here and in the hardware interface unless double compensation is intended.

## Cartesian and posture objectives

For all end effectors, the controller stacks world-aligned Jacobians into $\mathbf{J}$. With pose error $\mathbf{e}$ and task velocity $\mathbf{J}\dot{\mathbf{q}}$, it forms

$$
\ddot{\mathbf{x}}_{\text{cmd}}
= \mathbf{K}_{p,x}\mathbf{e}
+ \mathbf{K}_{d,x}(\dot{\mathbf{x}}_{\text{target}}-\mathbf{J}\dot{\mathbf{q}}).
$$

`task.error_clip` limits the three translational and three rotational error components before applying the gains. A task axis whose proportional and derivative gains are both zero is removed from the stacked QP task.

The secondary posture reference is

$$
\ddot{\mathbf{q}}_{\text{ref}}
= \mathbf{K}_{p,q}(\mathbf{q}_{\text{target}}-\mathbf{q})
+ \mathbf{K}_{d,q}(\dot{\mathbf{q}}_{\text{target}}-\dot{\mathbf{q}}).
$$

### Nullspace projector

`nullspace.projector_type` selects how the posture objective interacts with the Cartesian objective:

- `dynamic`: dynamically consistent torque projector; normally preferred for torque control.
- `kinematic`: uses $\mathbf{I}-\mathbf{J}^{+}\mathbf{J}$.
- `none`: uses the identity, so the posture and Cartesian objectives may compete directly.

`none` does not disable posture control. Set `weights.qddot: 0.0` to remove the posture objective.

## QP formulation

The decision variable is motor torque with model compensation removed:

$$
\boldsymbol{\tau}_m = \boldsymbol{\tau}-\mathbf{h},
\qquad
\ddot{\mathbf{q}}=\mathbf{M}^{-1}\boldsymbol{\tau}_m.
$$

The controller minimizes Cartesian acceleration error, projected posture acceleration error, torque regularization, and generalized-acceleration regularization:

$$
\begin{aligned}
\min_{\boldsymbol{\tau}_m}\;&
w_x\left\|\mathbf{J}\mathbf{M}^{-1}\boldsymbol{\tau}_m
  -(\ddot{\mathbf{x}}_{\text{cmd}}-\dot{\mathbf{J}}\dot{\mathbf{q}})\right\|^2 \\
&+w_q\left\|\mathbf{M}^{-1}\mathbf{N}_\tau\boldsymbol{\tau}_m
  -\mathbf{M}^{-1}\mathbf{N}_\tau\mathbf{M}\ddot{\mathbf{q}}_{\text{ref}}\right\|^2 \\
&+w_\tau\|\boldsymbol{\tau}_m\|^2
+w_a\|\mathbf{M}^{-1}\boldsymbol{\tau}_m\|^2,
\end{aligned}
$$

subject to

$$
\boldsymbol{\tau}_{\min}-\mathbf{h}
\leq \boldsymbol{\tau}_m
\leq \boldsymbol{\tau}_{\max}-\mathbf{h}.
$$

The WBC feed-forward command is $\boldsymbol{\tau}_{\text{ff}}=\boldsymbol{\tau}_m+\mathbf{h}$.

## Optional direct joint feedback

When `feedback.enabled` is true, the controller adds direct joint PD feedback:

$$
\boldsymbol{\tau}_{\text{fb}} =
\mathbf{K}_{p,\text{fb}}(\mathbf{q}_{\text{target}}-\mathbf{q})+
\mathbf{K}_{d,\text{fb}}(\dot{\mathbf{q}}_{\text{target}}-\dot{\mathbf{q}}).
$$

When `feedback.use_acc_integration` is true, the direct-feedback targets are replaced by a
one-step constant-acceleration prediction from the measured joint state:

$$
\begin{aligned}
\ddot{\mathbf{q}}_{\text{target}} &= \mathbf{M}^{-1}\boldsymbol{\tau}_m, \\
\dot{\mathbf{q}}_{\text{target}} &= \dot{\mathbf{q}} + \ddot{\mathbf{q}}_{\text{target}}\Delta t, \\
\mathbf{q}_{\text{target}} &= \mathbf{q} + \dot{\mathbf{q}}\Delta t
  + \tfrac{1}{2}\ddot{\mathbf{q}}_{\text{target}}\Delta t^2.
\end{aligned}
$$

The prediction is recomputed from measured state every update rather than accumulated across
updates. Here $\boldsymbol{\tau}_m$ is motion torque only; nonlinear compensation is excluded.
The QP posture objective is computed before this prediction and still uses the nominal or received
joint target. When `use_acc_integration` is false, direct feedback uses that posture target directly.

The raw total is $\boldsymbol{\tau}_{\text{ff}}+\boldsymbol{\tau}_{\text{fb}}$. Torque-rate limiting, output filtering, and the final torque clamp are applied to that sum. These feedback gains are torque gains and are independent of `posture.kp` and `posture.kd`, which create an acceleration reference inside the QP. Direct feedback is not nullspace-projected and can therefore compete with Cartesian tracking.

## Torque decomposition diagnostics

Set `decompose_commands.enabled: true` to publish `sensor_msgs/msg/JointState` messages. Joint names are in `name`, and each torque vector is in `effort`:

| Private topic | Published torque |
| --- | --- |
| `~/decompose_commands/motion` | QP motor torque $\boldsymbol{\tau}_m$ |
| `~/decompose_commands/nonlinear` | Model compensation $\mathbf{h}$ |
| `~/decompose_commands/feedforward` | $\boldsymbol{\tau}_m+\mathbf{h}$ |
| `~/decompose_commands/feedback` | Direct joint PD feedback |
| `~/decompose_commands/command` | Final rate-limited, filtered, and clamped hardware command |

All five messages from a control cycle have the same timestamp. Publication uses nonblocking real-time publishers and is rate-limited by `decompose_commands.publish_frequency`.

## Complete configuration example

```yaml
whole_body_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
    end_effector_frames: [tool_frame]
    base_frame: base_link

    topics:
      target_pose: [~/target_pose/tool]
      target_velocity: []  # Missing Cartesian velocity defaults to zero.
      target_joint: ~/target_joint

    task:
      kp: [400.0, 400.0, 400.0, 40.0, 40.0, 40.0]
      kd: [40.0, 40.0, 40.0, 12.0, 12.0, 12.0]
      error_clip: [0.10, 0.10, 0.10, 0.50, 0.50, 0.50]

    posture:
      nominal: [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
      kp: [10.0]
      kd: [1.0]

    feedback:
      enabled: false
      use_acc_integration: false
      kp: [0.0]
      kd: [0.0]

    dynamics:
      armature: [0.0]
      joint_damping: [0.0]
      use_coriolis: true
      use_gravity: true

    weights:
      task: 10.0
      qddot: 10.0
      regularization: 0.001
      regularization_qddot: 1.0

    nullspace:
      regularization: 0.01
      projector_type: dynamic

    torque_limits: []
    max_delta_tau: 0.5
    filter:
      target_pose: 0.1
      output_torque: 0.5
    qp:
      max_working_set_recalculations: 200
    decompose_commands:
      enabled: false
      publish_frequency: 100.0
      motion_topic: ~/decompose_commands/motion
      nonlinear_topic: ~/decompose_commands/nonlinear
      feedforward_topic: ~/decompose_commands/feedforward
      feedback_topic: ~/decompose_commands/feedback
      command_topic: ~/decompose_commands/command
    stop_commands: false
    log:
      enabled: false
```

## Parameter reference

| Parameter | Accepted value | Meaning |
| --- | --- | --- |
| `joints` | nonempty string array | Ordered controlled scalar joints and hardware-interface order. |
| `end_effector_frames` | nonempty string array | Ordered Pinocchio frames for independent 6D tasks. |
| `base_frame` | string | Expected pose-message frame; empty disables validation. |
| `topics.target_pose` | one suffix per end effector | Creates `~/target_pose/<suffix>`. Values must be unique and nonempty. |
| `topics.target_velocity` | empty, or one optional topic per end effector | Matching Cartesian velocity targets; missing or mismatched input means zero velocity. |
| `topics.target_joint` | nonempty suffix | Creates `~/<suffix>` for optional posture commands. |
| `task.kp`, `task.kd` | 6 values or 6 per task | Cartesian acceleration gains in `[x,y,z,rx,ry,rz]` order. |
| `task.error_clip` | 6 nonnegative values | Shared absolute Cartesian error limits for every task. |
| `posture.nominal` | 1 value or one per joint | Fallback joint position when no command supplies that joint; target velocity is zero. |
| `posture.kp`, `posture.kd` | 1 value or one per joint | Joint-posture acceleration gains. |
| `feedback.enabled` | boolean | Adds direct joint PD torque to the WBC feed-forward torque. |
| `feedback.use_acc_integration` | boolean | Uses a one-step prediction from $\mathbf{M}^{-1}\boldsymbol{\tau}_m$ as the direct-feedback target. |
| `feedback.kp`, `feedback.kd` | 1 value or one per joint | Direct joint torque-feedback gains. |
| `dynamics.armature` | 1 value or one per joint | Nonnegative diagonal inertia added to $\mathbf{M}$. |
| `dynamics.joint_damping` | 1 value or one per joint | Nonnegative viscous damping coefficient. |
| `dynamics.use_coriolis` | boolean | Enables $\mathbf{C}\dot{\mathbf{q}}$. |
| `dynamics.use_gravity` | boolean | Enables $\mathbf{g}$. |
| `weights.task` | nonnegative | Cartesian objective weight. |
| `weights.qddot` | nonnegative | Posture objective weight; zero disables it. |
| `weights.regularization` | positive | Torque Hessian regularization. |
| `weights.regularization_qddot` | nonnegative | Generalized-acceleration regularization. |
| `nullspace.regularization` | positive | Damping for task pseudoinverses. |
| `nullspace.projector_type` | `dynamic`, `kinematic`, `none` | Posture projector selection. |
| `torque_limits` | empty, 1 value, or one per joint | Symmetric positive limits; empty reads URDF effort limits. |
| `max_delta_tau` | nonnegative | Maximum torque change per update; zero disables rate limiting. |
| `filter.target_pose` | $[0,1]$ | Previous-sample weight for pose filtering; zero means no filtering. |
| `filter.output_torque` | $[0,1]$ | Previous-sample weight for torque filtering; zero means no filtering. |
| `qp.max_working_set_recalculations` | positive integer | qpOASES iteration budget per update. |
| `decompose_commands.enabled` | boolean | Enables torque-component diagnostic topics. |
| `decompose_commands.publish_frequency` | positive Hz | Diagnostic publication rate. |
| `decompose_commands.*_topic` | nonempty topic name | Topics for motion, nonlinear, feed-forward, feedback, and final command torque. |
| `stop_commands` | boolean | Computes normally but suppresses hardware command writes. |
| `log.enabled` | boolean | Enables throttled torque and first-task-error logging. |

## Recommended commissioning sequence

1. Verify joint and frame names against `robot_description`.
2. Start with `stop_commands: true` and confirm that configuration and state reads succeed.
3. Verify whether the hardware already compensates gravity or Coriolis effects before enabling those terms.
4. Start with conservative task and posture gains, explicit torque limits, and torque-rate limiting.
5. Command a hold pose before trying moving trajectories.
6. Enable one Cartesian task or axis at a time, then add posture control and additional end effectors.
