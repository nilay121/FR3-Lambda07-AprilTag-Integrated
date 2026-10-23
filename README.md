# FR3 + lambda.7 teleoperation and data logging

ROS 2 tooling to drive a Franka Research 3 with a Force Dimension lambda.7 haptic
device, reflect the arm's external force back to the operator's hand, and record
everything to a single time-aligned CSV. An optional RealSense + AprilTag branch
adds object poses to the same recording, so a demonstration contains the robot,
the operator's input, and the scene in one file.

This is bring-up and data-collection code for a research setup, not a product.
The control loop is Python and deliberately conservative: low gains, small step
limits, freshness watchdogs on every input, and both orientation and force
feedback off unless you explicitly ask for them.

## What it was built and run on

| | |
|---|---|
| Arm | Franka Research 3, FCI enabled, `172.16.0.30` on the lab network |
| Haptic device | Force Dimension lambda.7 (USB `1451:040c`), 6-DOF motion + gripper + 3-DOF force |
| Camera (optional) | Intel RealSense D4xx, AprilTag 36h11 family |
| OS / ROS | Ubuntu 24.04, ROS 2 Jazzy |
| Franka stack | `franka_ros2` v3.4.1, libfranka 0.20.5, with a two-line patch (see `patches/`) |
| Haptic stack | `forcedimension_ros2` (Kinesis fork of the ICube-Robotics driver) |
| Python | rclpy, numpy, scipy, pandas (analysis only) |

Three separate colcon workspaces are involved. This repository holds the parts
that are ours: the bridge, the loggers, the perception helpers, the lambda.7
bringup package, and the patch against the Franka examples. The upstream
workspaces are cloned during setup rather than vendored here.

## Layout

```
teleop/         the lambda.7 -> FR3 bridge, plus an optional raw-SDK force publisher
logging/        synchronised CSV logging for FR3 + lambda.7 (+ AprilTag), and a log checker
perception/     RealSense + AprilTag detection, visualiser, and camera-to-base calibration
lambda07/       lambda.7 bringup package, .repos manifests, udev rule installer
patches/        the franka_example_controllers patch this setup depends on
scripts/        setup_env.sh, sourced at the top of every terminal
data/sample/    a short example recording, for the column layout
docs/           setup, bring-up, teleoperation, logging, calibration, troubleshooting
```

Each folder has its own README with the short version; `docs/` has the long one.

## Where to start

1. [docs/01_installation.md](docs/01_installation.md) — ROS 2, the three workspaces,
   the Franka patch, the udev rule, and the optional camera stack.
2. [docs/02_bringup.md](docs/02_bringup.md) — the terminal-by-terminal start order
   and what to check after each step.
3. [docs/03_teleoperation.md](docs/03_teleoperation.md) — running the bridge, the
   staged translation → orientation → force procedure, and stiffness tuning.
4. [docs/04_logging.md](docs/04_logging.md) — synchronised logging: what a row means,
   the full column schema, two-signal vs three-signal recording, and how to check a file.
5. [docs/05_apriltag_calibration.md](docs/05_apriltag_calibration.md) — optional;
   only needed if you want tag poses in the robot base frame.
6. [docs/06_troubleshooting.md](docs/06_troubleshooting.md) — the failures that
   actually came up, and what fixed them.

The camera branch is genuinely optional. Everything except tag columns works with
`--no-tags` (standalone logger) or `--no-require-apriltag` (bridge) and no camera
plugged in.

## Status

Working and used for data collection:

- 3-DOF relative translation control with a keyboard clutch, workspace limit and
  per-cycle step limit.
- Relative orientation control behind `--orientation-enabled`, scaled and
  angle-clamped.
- Franka Hand open/close from the lambda.7 gripper axis.
- 3-axis translational force feedback behind `--force-enabled`, with a bias
  estimate and a hard saturation.
- Single-CSV logging of arm state, lambda.7 state and AprilTag poses on one clock.

## Not covered / not in this repository

Known bugs

- `gripper_width` is 100% NaN in every recording in `data/`. The bridge defaults to
  `--gripper-topic /fr3_gripper/joint_states` while its gripper actions are under
  `/franka_gripper/`. Pass the real name: `--gripper-topic /franka_gripper/joint_states`.
- `lambda_fr3_bridge.py` hard-codes the tag TF parent frame to `base` and ignores
  `--camera-frame`. Tag columns are all NaN unless
  `perception/calibration/camera_to_robot_tf.py` is running.

Missing capabilities

- No torque feedback to the lambda.7 wrist — `_publish_force` pads those slots with
  zero. Six-axis haptics needs the FR3 torque mapped into rotational commands and a
  driver that exposes them.
- Grasp force is not measured anywhere. The stock Franka Hand reports width with zero
  effort, so `gripper_closed_cmd` is a command, not a reading.
- No per-cycle rate limit on orientation. Translation is bounded by `--max-step`; the
  target quaternion can move the full `--max-angle` in one cycle.
- `Rmap` is identity and uncalibrated, and applies to translation and force only — the
  orientation delta is composed in the FR3 end-effector body frame, so it will not
  follow `Rmap` once you change it.
- No real-time control path. The bridge is a ~500 Hz Python loop, adequate for low-gain
  bring-up only; a high-gain or networked bilateral loop needs a C++ controller with
  passivity or energy-tank protection.
- No camera images or depth in the recordings, only tag poses. Depth streams but is unused.
- No controller internals in the CSV (stiffness, commanded equilibrium pose). Record a
  rosbag alongside if you need them.
