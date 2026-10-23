# perception/ (optional)

RealSense + AprilTag detection and the camera-to-robot calibration. Everything
else in this repository works without it; skip it unless you want object poses in
your recordings.

```
launch/apriltag_realsense.launch.py   apriltag_ros with the RealSense topics remapped
launch/apriltag_viz.launch.py         the above plus the live overlay
scripts/apriltag_viz.py               OpenCV overlay: tag id, bounding box, position, RPY
calibration/pose_transform.py         three measured points on a tag -> its pose in the base frame
calibration/camera_to_robot_tf.py     solves and broadcasts base -> camera_link, averaged over 50 samples
calibration/calibration_pts.example.txt   a worked example of the measurements from step 1
```

These are run by path, not installed as a ROS package.

```bash
ros2 launch realsense2_camera rs_launch.py
ros2 launch perception/launch/apriltag_realsense.launch.py
python3 perception/calibration/camera_to_robot_tf.py     # leave running all session
ros2 run tf2_ros tf2_echo base tag36h11:12               # verify
```

Two things to know before using it:

- Tag poses are published to **TF** as frames `<family>:<id>` (e.g. `36h11:12`),
  not to `/detections`. That topic carries 2D data only.
- The numbers hard-coded in `camera_to_robot_tf.py` and `pose_transform.py` are
  from one specific camera mounting. They are committed as a worked example.
  Redo the measurement for your own setup, and again whenever the camera moves.

Procedure: [../docs/05_apriltag_calibration.md](../docs/05_apriltag_calibration.md).
