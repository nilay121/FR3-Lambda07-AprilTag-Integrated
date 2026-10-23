# 5. RealSense, AprilTag and camera-to-base calibration

Optional. Skip it unless you want object poses in your recordings.

Two separate things live in `perception/`:

- **Detection** — `apriltag_ros` reading the RealSense colour stream and
  publishing tag poses to TF as frames named `<family>:<id>`, e.g. `36h11:12`.
- **Calibration** — a static transform from the robot base to the camera, so
  those tag poses can be read in the robot's frame instead of the camera's.

Detection alone is enough if you are happy with poses in the camera optical
frame. The bridge's logging path expects the base frame, so for the standard
workflow you need both.

## 5.1 Detection

```bash
ros2 launch realsense2_camera rs_launch.py
ros2 launch perception/launch/apriltag_realsense.launch.py
```

The custom launch file exists because the stock `apriltag_ros` launch subscribes
to topic names the RealSense driver does not publish. It remaps `image_rect` to
`/camera/camera/color/image_raw` and `camera_info` to
`/camera/camera/color/camera_info`, and loads the stock 36h11 config.

Checks:

```bash
ros2 topic hz /detections                  # ~camera rate when a tag is in view
ros2 run tf2_tools view_frames             # writes a PDF of the TF tree
ros2 run rqt_image_view rqt_image_view     # pick /camera/camera/color/image_raw
```

Which ids are detected is set in `/opt/ros/jazzy/share/apriltag_ros/cfg/tags_36h11.yaml`,
not in our launch file. Add every id you plan to use, including the reference tag.
Tag size matters: a wrong `size` in that config gives poses that look plausible
and are wrong by a scale factor.

A live overlay with the tag id, position and roll/pitch/yaw drawn on the video:

```bash
python3 perception/scripts/apriltag_viz.py
# or, detector and overlay together:
ros2 launch perception/launch/apriltag_viz.launch.py
```

Use it before recording to confirm every tag in the scene is actually seen from
the camera's position, and stays seen through the motion you are about to demo.
`check_log.py` reporting a tag at 50% visibility afterwards means this step got
skipped.

## 5.2 Calibrating camera to robot base

The approach: place one AprilTag (id 0 by default) somewhere fixed and visible,
measure where it is in the robot base frame by touching it with the end effector,
then let the camera see it and solve for the camera pose.

```
        U
        ●
  ┌───────────┐
  │           │
  │     C     ●  R
  │           │
  └───────────┘
```

**Step 1 — measure three points on the reference tag with the arm.** Move the end
effector to the tag centre `C`, then to the midpoint of its right edge `R`, then
to the midpoint of its top edge `U`. At each one:

```bash
ros2 topic echo /franka_robot_state_broadcaster/current_pose --once \
  --qos-reliability best_effort
```

Write down the three positions. `perception/calibration/calibration_pts.example.txt`
is a filled-in example from this lab.

Accuracy here sets the accuracy of everything downstream. Use the same point on
the end effector each time and take the readings carefully; a couple of
millimetres of sloppiness in `C`, `R` and `U` turns into a visible orientation
error in the fitted tag frame, because the axes are built from the differences
between those points.

**Step 2 — turn the three points into a pose.** Put them into
`perception/calibration/pose_transform.py` (`center`, `right`, `top` near the
top of the file) and run it:

```bash
python3 perception/calibration/pose_transform.py
```

It builds the tag's X axis from `C → R`, its Y axis from `C → U`, orthogonalises
into a right-handed frame, and prints the tag position and quaternion in the base
frame. It also publishes `base → reference_tag_0_known` as a static transform so
you can eyeball it in RViz against the detected tag.

**Step 3 — put those numbers into the camera node.** Copy the printed position
and quaternion into `tag_position` and `tag_quaternion` in
`perception/calibration/camera_to_robot_tf.py`.

**Step 4 — run the camera transform publisher.**

```bash
python3 perception/calibration/camera_to_robot_tf.py
```

It waits until it can see `camera_color_optical_frame → tag36h11:0`, then chains

```
base → tag (known, from step 3)
tag  → camera_color_optical_frame (observed, inverted)
     → camera_link (RealSense internal transform, inverted)
```

to get `base → camera_link`, which it broadcasts as a static transform. It
averages 50 observations first — translations by mean, rotations by mean matrix
followed by an SVD re-orthogonalisation — and logs progress every ten samples.
After it converges it keeps republishing the same fixed transform, so a
subsequent detection dropout does not move the frame.

**Step 5 — verify.**

```bash
ros2 run tf2_ros tf2_echo base tag36h11:12
```

Put a tag somewhere you can measure by hand and check the printed position
against a ruler. Do this before recording, not after.

## 5.3 Operational rules

- Leave `camera_to_robot_tf.py` running for the entire session. The bridge looks
  tag poses up in `base`; without this node every tag column is NaN and nothing
  reports an error.
- The reference tag must be visible when the node starts. Once it has converged
  the tag can be occluded or removed.
- Moving or bumping the camera invalidates the calibration. Redo steps 1–4. There
  is no drift detection.
- The numbers currently in `camera_to_robot_tf.py` and `pose_transform.py` are
  from one specific camera mounting in one lab. They are committed as a worked
  example, not as defaults for your setup.
- The frame is `base`, not `fr3_link0`. They coincide in the standard Franka
  description, but if your TF tree names things differently, adjust
  `message.header.frame_id` in `camera_to_robot_tf.py` and the parent frame in
  the logger.

## 5.4 Depth

The pipeline uses the colour stream only. Depth is streaming and available on
`/camera/camera/depth/image_rect_raw`, but nothing in this repository reads it.
If you want RGB-D rather than tag poses, record a bag of the camera topics
alongside the CSV.
