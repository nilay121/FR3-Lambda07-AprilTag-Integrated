# 3. Teleoperation

`teleop/lambda_fr3_bridge.py` is one node that closes the loop between the
lambda.7 and the FR3. It reads the haptic device, publishes a Cartesian target
to the impedance controller, drives the Franka Hand, reflects the arm's external
force back to the handle, and records a CSV while it runs.

## 3.1 Topics it touches

| Signal | Topic | QoS |
|---|---|---|
| FR3 end-effector pose | `/franka_robot_state_broadcaster/current_pose` | best effort |
| FR3 external wrench | `/franka_robot_state_broadcaster/external_wrench_in_base_frame` | best effort |
| FR3 arm joints | `/joint_states` (names starting `fr3_joint`) | reliable |
| Franka Hand width | `/fr3_gripper/joint_states` — see the note in the root README | reliable |
| FR3 target | `/cartesian_impedance_example_controller/equilibrium_pose` | reliable |
| Franka Hand actions | `/franka_gripper/grasp`, `/franka_gripper/move` | action |
| lambda.7 pose | `/fd/ee_pose` | reliable |
| lambda.7 joints | `/fd/joint_states` (`fd_roll/pitch/yaw`, `fd_clutch`, efforts) | reliable |
| lambda.7 force command | `/fd/fd_controller/commands` | reliable |
| AprilTag detections | `/detections` (`apriltag_msgs/AprilTagDetectionArray`) | reliable |
| AprilTag poses | TF frames `<family>:<id>`, e.g. `36h11:12` | — |

Two details that cost time if you don't know them:

- The device orientation the bridge uses comes from the `fd_roll` / `fd_pitch` /
  `fd_yaw` **joint positions** in `/fd/joint_states`, composed as an XYZ Euler
  triple — not from the quaternion in `/fd/ee_pose`. Launch `fd_bringup` with
  `use_orientation:=true` or those joints are absent and orientation control is
  silently inert.
- Tag poses are not in `/detections`. That message carries 2D data only (id,
  family, corners, centre, homography). `apriltag_ros` publishes each tag's
  6-DOF pose to TF, so the bridge reads them with a TF listener.

## 3.2 Control loop

Running at `--rate` (default 500 Hz):

1. **Freshness gates.** The lambda.7 pose and the FR3 pose must both be newer
   than `--watchdog` (0.10 s). If either goes stale, teleoperation stops itself,
   zero force is commanded, and the last target is held. The wrench has its own
   `--wrench-watchdog`.
2. **Engagement.** On the first fresh cycle after you press `s`, the bridge
   latches four origins: lambda position and orientation, FR3 position and
   orientation. Everything after that is relative to those, so the arm never
   jumps to an absolute device position.
3. **Translation.** `target = pF0 + scale · Rmap · (p_lambda − pL0)`, then
   clamped into a sphere of radius `--workspace` around the engagement point,
   then rate-limited to `--max-step` metres per cycle.
4. **Orientation** (only with `--orientation-enabled`).
   `q_target = qF0 · scaled(qL0⁻¹ · qL)`, with the delta angle scaled by
   `--rotation-scale` and saturated at `--max-angle` radians.
5. **Force** (only with `--force-enabled`).
   `f_lambda = −gain · Rmapᵀ · (wrench − bias)`, axis-wise deadband, then the
   vector magnitude saturated at `--max-force`.

The wrench bias is averaged from `--bias-samples` samples collected **only while
teleoperation is stopped**. At the default 500 samples and 500 Hz that is about
a second. Leave the arm stopped and unloaded for a couple of seconds after
starting the bridge, and wait for `wrench bias ready: [...] N` in the log before
pressing `s`. Force feedback stays at zero until the bias exists.

## 3.3 Operating it

- Lowercase `s` in the bridge terminal toggles teleoperation START/STOP.
- START also opens a CSV; STOP closes it. Each toggle produces a new file.
- The lambda.7 gripper axis drives the Franka Hand whenever the bridge is
  running: below `--grip-close-th` grasps, above `--grip-open-th` opens.
- Ctrl+C commands zero force and finalises the CSV before exiting.

Before the first run with force enabled, hold the lambda.7 handle. The device
renders force through its motors, and a badly signed mapping will move the
handle on its own.

## 3.4 Staged bring-up

Orientation and force are separate flags and both default to off. Do not turn on
two things at once — if the behaviour is wrong you will not know which one caused
it.

**Stage 1 — translation and gripper only.** Nothing extra on the command line.

```bash
python3 lambda_fr3_bridge.py --task-type bringup
```

Watch the target before letting the arm follow it:

```bash
ros2 topic echo /cartesian_impedance_example_controller/equilibrium_pose --field pose.position
```

Move the handle along one axis at a time and confirm the target moves along the
axis you expect. If it doesn't, that is `Rmap` (see 3.6), not a bug.

**Stage 2 — add orientation, force still off.**

```bash
python3 lambda_fr3_bridge.py --rate 100 --scale 5.0 --workspace 0.40 --max-step 0.0015 \
  --watchdog 0.50 --orientation-enabled --rotation-scale 0.5 --max-angle 0.35
```

Echo `--field pose.orientation` and confirm the quaternion moves smoothly and
stays normalised, then test one rotation axis at a time in free space.

**Stage 3 — translational force feedback at a fifth of nominal gain.**

```bash
python3 lambda_fr3_bridge.py --rate 100 --scale 5.0 --workspace 0.40 --max-step 0.0015 \
  --watchdog 0.50 --orientation-enabled --rotation-scale 0.5 --max-angle 0.35 \
  --force-enabled --force-gain 0.05 --max-force 0.5 --deadband 0.4 --force-dim 3
```

Push the FR3 end effector by hand. The handle must resist — push back against
your hand, not help it along. If the direction is wrong, stop immediately and fix
`Rmap` or the force sign. Never correct a sign problem by raising the gain.

**Stage 4 — raise the gain once the direction is verified**, in steps:
`--force-gain 0.10 --max-force 1.0`, then at most `--force-gain 0.15 --max-force 1.5`.

**The configuration used for data collection here**, after all four stages passed:

```bash
python3 lambda_fr3_bridge.py \
  --rate 500 --scale 5.0 --workspace 0.60 --max-step 0.0015 \
  --watchdog 0.10 --wrench-watchdog 0.10 \
  --orientation-enabled --rotation-scale 1.0 --max-angle 0.70 \
  --force-enabled --force-gain 0.9 --max-force 1.0 --deadband 0.05 --force-dim 3 \
  --grasp-action /franka_gripper/grasp --move-action /franka_gripper/move \
  --task-type all_tasks
```

Treat those numbers as one working point on one machine, not a default. Do not
start a new setup at `--force-gain 0.9`.

## 3.5 Flags

Motion and safety:

| Flag | Default | Meaning |
|---|---|---|
| `--rate` | 500 | control loop Hz |
| `--scale` | 0.25 | lambda displacement to FR3 displacement ratio |
| `--workspace` | 0.10 | radius in metres of the sphere the target stays inside |
| `--max-step` | 0.0005 | maximum target movement per cycle, metres |
| `--watchdog` | 0.10 | pose freshness timeout, seconds |
| `--wrench-watchdog` | 0.10 | wrench freshness timeout, seconds |
| `--base-frame` | `fr3_link0` | frame stamped on the published target |

Orientation:

| Flag | Default | Meaning |
|---|---|---|
| `--orientation-enabled` | off | enable relative orientation control |
| `--rotation-scale` | 0.5 | scaling of the rotation delta |
| `--max-angle` | 0.35 | delta angle saturation, radians (≈20°) |

Force feedback:

| Flag | Default | Meaning |
|---|---|---|
| `--force-enabled` | off | enable force reflection |
| `--force-gain` | 0.05 | N of handle force per N of arm force |
| `--max-force` | 0.8 | magnitude saturation, N |
| `--deadband` | 0.05 | per-axis deadband, N |
| `--force-dim` | 3 | command dimension; this driver setup uses 3 |
| `--bias-samples` | 500 | unloaded wrench samples averaged for the bias |

Gripper:

| Flag | Default | Meaning |
|---|---|---|
| `--gripper-joint` | `fd_clutch` | lambda.7 joint driving the hand |
| `--grip-close-th` | 0.03 | below this the hand grasps |
| `--grip-open-th` | 0.06 | above this the hand opens |
| `--grip-open-width` | 0.08 | open width, metres |
| `--grip-speed` | 0.05 | m/s |
| `--grip-force` | 20.0 | grasp force, N |

Logging flags are documented in [04_logging.md](04_logging.md). Every topic and
action name is also a flag, so nothing needs editing in the file to point the
bridge at differently-named topics.

## 3.6 The two things that must be right

**Apply the controller patch.** Without it the impedance controller overwrites
your target with its own motion generator and the arm ignores everything the
bridge publishes. See [01_installation.md](01_installation.md) §1.3.

**Calibrate `Rmap`.** It is the 3×3 lambda-to-FR3 axis mapping, hard-coded as
identity near the top of `Bridge.__init__`. Verify it one axis at a time in RViz
or by echoing the target before the arm is active. It applies to translation and
force only — the orientation delta is composed in the FR3 end-effector body frame
(`RF0 · R_delta`). While `Rmap` is identity the two agree trivially, but once you
change it, rotation will not follow. A lambda rotation about X turning the FR3
about the wrong axis is this frame choice, not arithmetic.

## 3.7 Alternative bridge variant

`teleop/lambda_fr3_bridge_split_csv.py` is an earlier version of the same bridge
that writes two files instead of one: a fixed-width `_state.csv` and a long-format
`_tags.csv` with one row per tag detection, joinable on `t`. Use it if a variable
number of tags per frame suits your analysis better than fixed columns. The
control code is otherwise the same. `lambda_fr3_bridge.py` is the one that
produced the recordings in `data/`.
