# 6. Troubleshooting

Failures that actually came up on this setup, and what fixed them.

## The arm ignores the bridge

`equilibrium_pose` shows your target moving, the controller is active, the arm
holds still.

The controller patch is not applied, or a rebuild reverted it. The stock
`cartesian_impedance_example_controller` calls `updateMotionTarget()` every
update and overwrites whatever you published.

```bash
cd ~/franka_ros2_ws/src
git diff --stat        # expect 2 files changed under franka_example_controllers
git apply /path/to/repo/patches/franka_example_controllers.patch   # if empty
cd ~/franka_ros2_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Rebuild and restart the controller — sourcing an old install tree gives the same
symptom.

## A topic exists but `ros2 topic echo` prints nothing

The two `franka_robot_state_broadcaster` convenience topics publish BEST_EFFORT.
A default (reliable) subscriber gets nothing, with no error.

```bash
ros2 topic echo /franka_robot_state_broadcaster/current_pose --qos-reliability best_effort
```

The bridge and both loggers already subscribe best-effort. If they see nothing,
it is not this.

## Gripper is "not connected", or gripper actions time out

Stop everything and clean up stale state:

```bash
pkill -f franka_gripper_node
pkill -f ros2_control_node
pkill -f robot_state_publisher
pkill -f "ros2 launch"

ros2 daemon stop
pkill -f _ros2_daemon

# only after every ROS node is stopped
sudo rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

Then open fresh terminals and source them identically (see
[02_bringup.md](02_bringup.md) §2.0). Deleting the Fast DDS shared-memory files
while a node is still running causes stranger failures than the one you started
with.

## `gripper_width` is all NaN in the logs

Known issue. The bridge subscribes to `/fr3_gripper/joint_states` by default,
while the gripper actions it uses are under `/franka_gripper/`. Check the real
name and pass it:

```bash
ros2 topic list | grep gripper
python3 teleop/lambda_fr3_bridge.py ... --gripper-topic /franka_gripper/joint_states
```

Same flag on `logging/sync_logger.py` and `logging/fr3_lambda7_logger.py`.

## Tag columns are all NaN

Work through it in this order:

1. Is the detector running? `ros2 topic hz /detections`
2. Is the id enabled in `/opt/ros/jazzy/share/apriltag_ros/cfg/tags_36h11.yaml`?
   An id missing there is never detected and nothing warns you.
3. Did you list the id in `--tag-ids`? Ids not listed are dropped silently.
4. Is `camera_to_robot_tf.py` running? The bridge looks tags up in the `base`
   frame; without that node the TF lookup fails for every tag.
   (`sync_logger.py --tag-frame ""` avoids the dependency.)
5. Do the TF frame names match? The lookup is `<family>:<id>`, e.g. `36h11:12`.
   Some `apriltag_ros` configurations publish `tag36h11:12` or a custom name.
   `ros2 run tf2_tools view_frames` shows what is actually there.

A tag that appears in `/detections` but not in the CSV is almost always 4 or 5.

## Bridge refuses to log

```
CSV logging: no publisher on /detections; is the RealSense AprilTag pipeline running?
```

By design — it will not start a recording that was meant to have tags and won't.
Either start the camera pipeline, or record without it:

```bash
python3 teleop/lambda_fr3_bridge.py --no-require-apriltag
```

## Orientation control does nothing

`fd_bringup` was launched without `use_orientation:=true`, so `/fd/joint_states`
has no `fd_roll` / `fd_pitch` / `fd_yaw`. The bridge reads the device orientation
from those joint positions, not from the quaternion in `/fd/ee_pose`.

```bash
ros2 topic echo /fd/joint_states --once     # look for fd_roll, fd_pitch, fd_yaw
```

If those joints exist but the quaternion in `/fd/ee_pose` never changes when you
twist the handle, that is a driver problem — sort it out before involving the
bridge.

## Force pushes the wrong way

Stop the bridge. Do not raise the gain to compensate.

The sign convention is `f_lambda = −gain · Rmapᵀ · (wrench − bias)`. Wrong
direction means `Rmap` does not match your physical axis alignment, or the
device frame convention is different from what was assumed. Fix `Rmap` in
`Bridge.__init__`, verify one axis at a time in free space at `--force-gain 0.05
--max-force 0.5`, then raise the gain.

## Force feedback stays at zero

The bias is not ready. It is collected only while teleoperation is **stopped**,
`--bias-samples` samples of the unloaded wrench (1 s at the defaults). Leave the
arm stopped and untouched for a couple of seconds after starting the bridge, and
wait for:

```
wrench bias ready: [...] N
```

The other gates that force zero output: `--force-enabled` not passed, the wrench
older than `--wrench-watchdog`, or a non-finite wrench sample.

## Teleoperation stops by itself

```
STOP: stale Lambda or Franka pose data
```

One of the two pose streams went older than `--watchdog` (0.10 s). Usual causes:
the lambda.7 launch died, the arm dropped out of FCI, or the machine is loaded
enough that a 500 Hz Python loop is missing its deadline. Check `ros2 topic hz`
on both poses. Raising `--watchdog` hides the symptom without fixing anything —
during bring-up on a busy machine `--watchdog 0.50` with `--rate 100` is a
reasonable trade, but not for real runs.

## Nothing talks to anything

Different `ROS_DOMAIN_ID` in different terminals, a conda Python shadowing the
system rclpy, or one terminal sourcing a stale workspace. Print the environment
in every terminal and compare:

```bash
conda deactivate 2>/dev/null || true
echo "RMW=$RMW_IMPLEMENTATION  DOMAIN=$ROS_DOMAIN_ID  PY=$(which python3)"
```

## lambda.7 needs sudo

The udev rule is missing or the device was not reconnected after installing it.

```bash
sudo lambda07/scripts/install_udev_rules.sh
# unplug, replug
lsusb | grep -i '1451:040c'
```

If it still needs root, check the session owns the device:

```bash
getfacl /dev/bus/usb/<bus>/<device>
```

## Two controllers fail to activate

`move_to_start_example_controller` and `cartesian_impedance_example_controller`
claim the same joint interfaces. Deactivate one before activating the other:

```bash
ros2 control list_controllers
ros2 control set_controller_state move_to_start_example_controller inactive
ros2 control load_controller --set-state active cartesian_impedance_example_controller
```

## Motion is rough or the arm feels stiff

Lower the impedance gains before touching anything in the bridge:

```bash
ros2 param set /cartesian_impedance_example_controller translational_stiffness 500.0
ros2 param set /cartesian_impedance_example_controller rotational_stiffness 25.0
```

If it is jitter rather than stiffness, check the loop is actually running at
`--rate` (a loaded machine or a slow logging path will not keep 500 Hz), and
confirm the RT kernel and `ulimit -r 99` are in place.
