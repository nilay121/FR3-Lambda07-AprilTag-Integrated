# 1. Installation

Target: Ubuntu 24.04 with ROS 2 Jazzy. Three colcon workspaces end up on the
machine, kept separate on purpose so a rebuild of one does not disturb the others:

```
~/franka_ros2_ws     franka_ros2 v3.4.1 + libfranka, patched
~/lambda-07_ws       forcedimension_ros2 + fd_sdk_vendor + lambda07_bringup
~/fr3_ws             RealSense + AprilTag (optional)
```

Clone this repository anywhere; it holds scripts you run, not packages you build,
apart from `lambda07/lambda07_bringup`.

## 1.1 ROS 2 Jazzy

```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions python3-vcstool
ros2 --version
```

Python packages the scripts need:

```bash
sudo apt install python3-numpy python3-scipy python3-pandas python3-opencv
```

Do **not** put `source /opt/ros/jazzy/setup.bash` plus every workspace into
`~/.bashrc` and forget about it. Mixing workspaces silently is the source of most
"the topic exists but nothing arrives" problems. Source explicitly per terminal;
see [02_bringup.md](02_bringup.md).

If you use conda, `conda deactivate` before sourcing ROS. A conda Python on the
path will shadow the system rclpy.

## 1.2 Real-time kernel and FCI

The FR3 needs a real-time kernel and a `limits.conf` entry for the control user
before `franka_bringup` will hold 1 kHz. Follow the Franka documentation for the
RT patch, then confirm:

```bash
uname -a | grep -i preempt          # should show PREEMPT_RT
ulimit -r                           # should be 99, not 0
ping -c 3 172.16.0.30               # the arm, from the control PC
```

Enable FCI in Desk (`https://172.16.0.30`), unlock the joints, and release the
external activation device before any controller is loaded. `franka_ros2/src` also
ships a `limits.conf` you can copy to `/etc/security/limits.conf`.

## 1.3 franka_ros2 with the local patch

```bash
mkdir -p ~/franka_ros2_ws && cd ~/franka_ros2_ws
git clone https://github.com/frankarobotics/franka_ros2.git src
cd src
git checkout v3.4.1
vcs import . < dependency.repos
```

Now apply the patch from this repository. It changes two files in
`franka_example_controllers` and it is not optional — without the first hunk the
bridge cannot command the arm at all.

```bash
cd ~/franka_ros2_ws/src
git apply /path/to/this/repo/patches/franka_example_controllers.patch
git diff --stat        # expect 2 files changed
```

What it does:

- `cartesian_impedance_example_controller.cpp` — comments out
  `updateMotionTarget(period.seconds(), target.orientation)`. The stock example
  overwrites the commanded target with its own internal motion generator, so
  anything published to `equilibrium_pose` is discarded. With the call removed,
  the controller tracks the target the bridge sends.
- `move_to_start_example_controller.cpp` — replaces the default start joint
  configuration with the pose used in this lab. Purely a convenience; edit the
  seven values to whatever start pose your workspace needs, or drop this hunk.

Build:

```bash
cd ~/franka_ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If you ever re-clone or re-import `franka_ros2`, re-apply the patch and rebuild.
A stock `franka_example_controllers` in the install tree looks identical from the
outside and fails in a way that is easy to misread as a bridge problem: the arm
holds position while `equilibrium_pose` shows your target moving.

## 1.4 lambda.7 workspace

Validate the device with the vendor SDK before involving ROS. It separates a
driver or USB problem from an integration problem.

```bash
cd ~/Lambda.07_USB/Linux/x86_64/sdk-3.17.7/bin
./HapticInit          # initialises and reports the device
./Emporium            # broader interactive check
```

If a demo reports a missing shared library, install what the error names and
retry (`ldd ./HapticInit | grep 'not found'` lists unresolved ones). If the demos
only work under `sudo`, install the udev rule below and reconnect the device.

```bash
sudo /path/to/this/repo/lambda07/scripts/install_udev_rules.sh
# unplug and replug the lambda.7, then rerun ./HapticInit without sudo
lsusb | grep -i '1451:040c'
```

Then the ROS workspace:

```bash
mkdir -p ~/lambda-07_ws && cd ~/lambda-07_ws
cp /path/to/this/repo/lambda07/config/lambda07_kinesis_https.repos .
vcs import src < lambda07_kinesis_https.repos
cp -r /path/to/this/repo/lambda07/lambda07_bringup src/

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source install/setup.bash
```

If you have the vendor SDK extracted locally, point the build at it instead of
letting `fd_sdk_vendor` fetch one:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DFD_SDK_ROOT=$HOME/Lambda.07_USB/Linux/x86_64/sdk-3.17.7 --symlink-install
```

Use `lambda07_kinesis_ssh.repos` instead if you have write access to the Kinesis
organisation and intend to push changes to the driver forks.

Smoke test without touching the hardware, then with it:

```bash
ros2 launch lambda07_bringup lambda07.launch.py use_fake_hardware:=true
ros2 launch lambda07_bringup lambda07.launch.py
```

`lambda07_bringup` is the read-only onboarding path (state, pose, clutch, RViz).
Teleoperation uses `fd_bringup` from the driver, which additionally exposes the
wrist axes and the force command interface — see [02_bringup.md](02_bringup.md).

## 1.5 RealSense and AprilTag (optional)

Skip this section entirely if you do not need object poses in the recordings.

Check the camera outside ROS first:

```bash
sudo apt install librealsense2-utils
realsense-viewer          # confirm RGB and depth streams
```

ROS packages, from apt:

```bash
sudo apt install ros-jazzy-librealsense2 ros-jazzy-realsense2-camera \
                 ros-jazzy-realsense2-description \
                 ros-jazzy-apriltag ros-jazzy-apriltag-msgs ros-jazzy-apriltag-ros
ros2 pkg list | grep apriltag
```

If the apt packages misbehave, build from source in `~/fr3_ws/src` instead:

```bash
mkdir -p ~/fr3_ws/src && cd ~/fr3_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
git clone https://github.com/christianrauch/apriltag_ros.git
cd ~/fr3_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

The launch files in `perception/launch/` are run directly with a path, not
installed as a package, so put this repository's `perception/` where you can
reach it and use the full path.

The stock `apriltag_ros` launch subscribes to topics the RealSense driver does
not publish under those names, which is why `perception/launch/apriltag_realsense.launch.py`
exists — it remaps `image_rect` and `camera_info` onto
`/camera/camera/color/image_raw` and `/camera/camera/color/camera_info`.

Which tag ids get detected is set in the detector config, not in our launch file:

```bash
sudo nano /opt/ros/jazzy/share/apriltag_ros/cfg/tags_36h11.yaml
```

Add every id you intend to place in the scene. An id missing from that file is
never detected, and the CSV column for it stays NaN with no error anywhere.
