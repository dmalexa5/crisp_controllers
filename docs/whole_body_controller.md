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
      target_pose: [left, right]
      target_joint: target_joint
```

For a controller named `whole_body_controller`, these settings create:

```text
/whole_body_controller/target_pose/left   geometry_msgs/msg/PoseStamped
/whole_body_controller/target_pose/right  geometry_msgs/msg/PoseStamped
/whole_body_controller/target_joint       sensor_msgs/msg/JointState
```

`topics.target_pose` and `end_effector_frames` are ordered lists with the same length. A pose target may use an empty `header.frame_id`, or it must equal `base_frame` when `base_frame` is configured. The controller does not transform targets between frames.

On activation, every Cartesian target and the posture target are initialized to the measured robot state. Consequently, publishing neither target type produces a hold command rather than motion toward a configuration from the YAML file. Named `JointState` posture commands may update a subset of controlled joints; unnamed commands follow the configured `joints` order.

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
- \mathbf{K}_{d,x}\mathbf{J}\dot{\mathbf{q}}.
$$

The current pose command contains no desired twist or acceleration feed-forward. `task.error_clip` limits the three translational and three rotational error components before applying the gains. A task axis whose proportional and derivative gains are both zero is removed from the stacked QP task.

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

The final command is $\boldsymbol{\tau}=\boldsymbol{\tau}_m+\mathbf{h}$, followed by torque-rate limiting, output filtering, and a final torque clamp.

## Complete configuration example

```yaml
whole_body_controller:
  ros__parameters:
    joints: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
    end_effector_frames: [tool_frame]
    base_frame: base_link

    topics:
      target_pose: [tool]
      target_joint: target_joint

    task:
      kp: [400.0, 400.0, 400.0, 40.0, 40.0, 40.0]
      kd: [40.0, 40.0, 40.0, 12.0, 12.0, 12.0]
      error_clip: [0.10, 0.10, 0.10, 0.50, 0.50, 0.50]

    posture:
      kp: [10.0]
      kd: [1.0]

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
| `topics.target_joint` | nonempty suffix | Creates `~/<suffix>` for optional posture commands. |
| `task.kp`, `task.kd` | 6 values or 6 per task | Cartesian acceleration gains in `[x,y,z,rx,ry,rz]` order. |
| `task.error_clip` | 6 nonnegative values | Shared absolute Cartesian error limits for every task. |
| `posture.kp`, `posture.kd` | 1 value or one per joint | Joint-posture acceleration gains. |
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
| `stop_commands` | boolean | Computes normally but suppresses hardware command writes. |
| `log.enabled` | boolean | Enables throttled torque and first-task-error logging. |

## Recommended commissioning sequence

1. Verify joint and frame names against `robot_description`.
2. Start with `stop_commands: true` and confirm that configuration and state reads succeed.
3. Verify whether the hardware already compensates gravity or Coriolis effects before enabling those terms.
4. Start with conservative task and posture gains, explicit torque limits, and torque-rate limiting.
5. Command a hold pose before trying moving trajectories.
6. Enable one Cartesian task or axis at a time, then add posture control and additional end effectors.

