# lambda07/

The parts of the lambda.7 workspace that are ours. The driver itself
(`forcedimension_ros2`, `fd_sdk_vendor`) is cloned during setup via the manifests
here rather than vendored.

```
lambda07_bringup/            ROS 2 package: launch, controller config, URDF, RViz
config/*.repos               vcs manifests for the Kinesis driver forks (https and ssh)
scripts/install_udev_rules.sh   udev rule so the device works without sudo
```

Setup:

```bash
mkdir -p ~/lambda-07_ws && cd ~/lambda-07_ws
cp /path/to/repo/lambda07/config/lambda07_kinesis_https.repos .
vcs import src < lambda07_kinesis_https.repos
cp -r /path/to/repo/lambda07/lambda07_bringup src/
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

USB access:

```bash
sudo scripts/install_udev_rules.sh      # then unplug and replug the device
lsusb | grep -i '1451:040c'
```

## Two launch paths, and which to use

`lambda07_bringup` is the read-only onboarding package: joint states, end-effector
pose, clutch button, RViz model. It does not enable force output. Use it to
confirm the device, the ROS graph and the description are healthy.

```bash
ros2 launch lambda07_bringup lambda07.launch.py use_fake_hardware:=true   # no hardware
ros2 launch lambda07_bringup lambda07.launch.py                           # real device
ros2 topic echo /lambda07/ee_pose
```

Teleoperation uses `fd_bringup` from the driver instead, which exposes the wrist
axes and the force command interface:

```bash
ros2 launch fd_bringup fd.launch.py use_orientation:=true use_clutch:=true
```

Both flags are required. Without `use_orientation` there are no `fd_roll`,
`fd_pitch`, `fd_yaw` joints and the bridge's orientation control is silently
inert; without `use_clutch` the gripper axis is missing.

Before either, validate the device with the vendor SDK demos (`HapticInit`,
`Emporium`) from the Force Dimension USB media. It separates a driver or USB
problem from an integration problem, and it takes two minutes.

## Note on force output

Force output through `lambda07_bringup` is not validated. On the workstation this
package was written for, activating the effort controller reached the hardware
but hit `std::out_of_range: unordered_map::at` and command-interface activation
failures before any nonzero command was sent. The force path that *is* in use for
teleoperation goes through `fd_bringup` and `/fd/fd_controller/commands`, driven
by the bridge — see [../docs/03_teleoperation.md](../docs/03_teleoperation.md),
and bring the gain up in stages.
