# 2. Bring-up order

Order matters. The bridge assumes the lambda.7 is publishing before it engages,
and the impedance controller must be active before anything is published to
`equilibrium_pose`. Start terminals in the order below and check each one before
opening the next.

## 2.0 Same environment in every terminal

Run this at the top of each terminal. Mismatched domain ids or a conda Python in
the path produce topics that exist but never deliver, which is a slow thing to
debug.

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
source ~/lambda-07_ws/install/setup.bash
export ROS_DOMAIN_ID=0
echo "RMW=$RMW_IMPLEMENTATION  DOMAIN=$ROS_DOMAIN_ID"
```

Every terminal must print the same two values. There is a copy of this in
`scripts/setup_env.sh`; `source` it rather than pasting.

## 2.1 Terminal 1 — lambda.7

```bash
cd ~/lambda-07_ws && source install/setup.bash
ros2 launch fd_bringup fd.launch.py use_orientation:=true use_clutch:=true
```

`use_orientation:=true` exposes the wrist axes, `use_clutch:=true` the gripper
axis. Without them the bridge sees a 3-DOF device and the gripper never moves.

Check before continuing:

```bash
ros2 control list_controllers -c /fd/controller_manager
ros2 topic echo /fd/ee_pose --field pose.position           # move the handle
ros2 topic echo /fd/ee_pose --field pose.orientation        # rotate it; must change
ros2 topic echo /fd/joint_states --once                     # fd_clutch present?
```

If the orientation quaternion is constant while you twist the handle, stop here.
Orientation teleoperation cannot work and enabling it later will look like a bug
in the bridge.

## 2.2 Terminal 2 — FR3 and gripper

```bash
cd ~/franka_ros2_ws && source install/setup.bash
ros2 launch franka_bringup franka.launch.py \
  robot_type:=fr3 robot_ip:=172.16.0.30 load_gripper:=true
```

Substitute your arm's address. Check:

```bash
ros2 topic hz /franka_robot_state_broadcaster/current_pose
ros2 topic list | grep gripper
```

`ros2 topic echo` on the two `franka_robot_state_broadcaster` convenience topics
needs best-effort QoS to show anything:

```bash
ros2 topic echo /franka_robot_state_broadcaster/current_pose --qos-reliability best_effort
```

This trips people up regularly. The topic is there, `ros2 topic info` shows a
publisher, and a default reliable subscriber receives nothing at all. The bridge
and the loggers already subscribe best-effort.

Optionally send the arm to a known start pose before teleoperating:

```bash
ros2 control load_controller --set-state active move_to_start_example_controller
```

The joint configuration it moves to is the one baked in by
`patches/franka_example_controllers.patch`. Deactivate it before loading the
impedance controller — two controllers claiming the same joints will fail to
activate.

## 2.3 Terminal 3 — Cartesian impedance controller

```bash
cd ~/franka_ros2_ws && source install/setup.bash
ros2 control load_controller --set-state active cartesian_impedance_example_controller
ros2 control list_controllers          # expect: active
```

Default stiffness is high for teleoperation. Softer values used here:

```bash
ros2 param set /cartesian_impedance_example_controller translational_stiffness 500.0
ros2 param set /cartesian_impedance_example_controller rotational_stiffness 25.0
```

Lower stiffness gives smoother teleoperation and a gentler response to a bad
target, at the cost of tracking accuracy. Change it before engaging, not during.

## 2.4 Terminal 4 — camera and AprilTag (optional)

Skip if you are recording without tags.

```bash
ros2 launch realsense2_camera rs_launch.py
```

In another terminal:

```bash
ros2 launch /path/to/repo/perception/launch/apriltag_realsense.launch.py
ros2 topic hz /detections
ros2 run tf2_tools view_frames        # tag frames appear as 36h11:<id>
```

If you want tag poses in the robot base frame rather than the camera optical
frame — which the bridge's default configuration expects — also run the camera
calibration publisher, and leave it running for the whole session:

```bash
python3 /path/to/repo/perception/calibration/camera_to_robot_tf.py
ros2 run tf2_ros tf2_echo base tag36h11:12
```

See [05_apriltag_calibration.md](05_apriltag_calibration.md) for how the numbers
in that script are obtained. They are specific to one camera mounting; a moved
camera invalidates them.

A live overlay is useful for checking tag visibility before recording:

```bash
python3 /path/to/repo/perception/scripts/apriltag_viz.py
```

## 2.5 Terminal 5 — bridge or logger

Teleoperation, which also records:

```bash
cd /path/to/repo/teleop
python3 lambda_fr3_bridge.py --task-type all_tasks
```

Recording only, no teleoperation:

```bash
cd /path/to/repo/logging
python3 sync_logger.py --task-type handguided --no-tags
```

Full flag sets and the staged procedure are in
[03_teleoperation.md](03_teleoperation.md) and [04_logging.md](04_logging.md).

## Shutdown

Ctrl+C the bridge first — it commands zero force to the lambda.7 and closes the
CSV cleanly. Then the controller terminal, then `franka_bringup`, then the
lambda.7 launch. Killing `fd_bringup` while the bridge is still commanding force
leaves the handle under whatever force was last rendered.
