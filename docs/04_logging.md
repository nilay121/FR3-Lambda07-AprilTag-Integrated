# 4. Synchronised logging

Everything in this repository writes the same CSV: one row per sample, one clock,
the robot and the haptic device and (optionally) the scene side by side. That is
the whole point — a demonstration is only useful later if the arm state, the
operator's input and the object poses can be read off the same row without
guessing at alignment.

There are three ways to produce one, all writing identical columns:

| Tool | Records | When to use |
|---|---|---|
| `teleop/lambda_fr3_bridge.py` | FR3 + lambda.7 + tags | while teleoperating; starts and stops with the `s` key |
| `logging/sync_logger.py` | FR3 + lambda.7, tags optional | any session with no bridge: hand-guided demos, `move_to_start` runs, gripper tests |
| `logging/fr3_lambda7_logger.py` | FR3 + lambda.7 only | older, narrower schema (includes EE velocity and full 6-DOF wrench, no tags); kept because some earlier data uses it |

`logging/record_rosbag.py` wraps `ros2 bag record` if you want raw topics instead.
It is not part of the CSV workflow.

## 4.1 What "synchronised" means here

One timer fires at the logging rate. Each tick reads the most recent value
received on every topic and writes them as one row with one timestamp, taken from
the ROS clock at the moment the row is built. Nothing is interpolated and nothing
is re-aligned afterwards.

So a row is a **zero-order hold** of each signal, not a simultaneous measurement.
The sources run at very different rates:

```
FR3 state       ~1 kHz     newest sample is at most ~1 ms old
lambda.7 state  ~1 kHz     at most ~1 ms old
camera + tags   ~30 Hz     up to ~33 ms old, and older if detection drops
```

At the 30 Hz default logging rate the robot and device columns are effectively
current. The tag columns are the loose ones: a tag pose can be up to one camera
frame old, and stays valid for up to `--det-timeout` (0.5 s) after the last
detection message. If you care about tight visual-motor alignment, raise the
camera rate rather than the logging rate — logging faster than the camera just
repeats the same tag values across more rows.

Missing data is written as `NaN`, never held indefinitely:

- The bridge writes NaN for any signal it has never received.
- `sync_logger.py` additionally writes NaN once a signal is older than
  `--max-age` (0.25 s), so a publisher that dies mid-recording shows up as a gap
  instead of a flat line that looks like a real measurement.
- A tag that is not visible in the current detection message gets NaN across its
  whole column block.

That last point makes NaN density a useful diagnostic. `check_log.py` reports it.

## 4.2 Column schema

37 state columns, then a fixed block of 8 columns per configured tag id.

| Columns | Meaning | Units / frame |
|---|---|---|
| `t` | ROS clock at the moment the row was built | seconds |
| `ee_x ee_y ee_z` | FR3 end-effector position | m, `fr3_link0` |
| `ee_qx ee_qy ee_qz ee_qw` | FR3 end-effector orientation | quaternion, `fr3_link0` |
| `q1 … q7` | arm joint angles | rad |
| `dq1 … dq7` | arm joint velocities | rad/s |
| `gripper_width` | Franka Hand opening, sum of the two finger joints | m |
| `gripper_closed_cmd` | 1 if the bridge has commanded a grasp, else 0 | — |
| `fd_x fd_y fd_z` | lambda.7 handle position | m, device frame |
| `fd_qx fd_qy fd_qz fd_qw` | lambda.7 handle orientation, from `fd_roll/pitch/yaw` | quaternion |
| `fd_Fx fd_Fy fd_Fz` | force the lambda.7 is rendering on the handle | N |
| `fr3_Fx fr3_Fy fr3_Fz` | FR3 external force estimate | N, `fr3_link0` |
| `tag<ID>_x/_y/_z` | tag position | m, see 4.4 |
| `tag<ID>_qx/_qy/_qz/_qw` | tag orientation | quaternion |
| `tag<ID>_decision_margin` | detector confidence for that tag in that frame | — |

`data/sample/teleop_sample_log.csv` is the first 400 rows of a real recording
with tags 10–14, if you want to look at the layout rather than read the table.

Two notes on interpreting the force columns:

- `fd_F*` is what the device's motors are **rendering**, not a load cell reading.
  In teleoperation that equals the reflected, gain-scaled FR3 wrench, so it is a
  record of what the operator felt, not an independent measurement of contact.
- `fr3_F*` is the arm's external wrench estimate, bias-corrected in the control
  path but logged raw. This is the physically meaningful object-interaction
  signal. Use it, not `fd_F*`, for anything about contact forces.
- `gripper_closed_cmd` is a command. The stock Franka Hand reports width with
  zero effort, so grasp force is not measured anywhere in this schema.

## 4.3 Tag columns are fixed at file creation

The tag column block is decided when the file is opened, from `--tag-ids`. This
keeps every row the same width and makes the file trivial to load, but it has two
consequences:

- An id you don't list is **silently dropped**, even if the detector sees it.
- An id you list but never place produces a column of NaN.

So set `--tag-ids` to the ids actually in your scene, every run:

```bash
--tag-ids 10 11 12 13 14        # the default in this repo
--tag-ids 0 1 5 9               # a different scene
```

Also make sure those ids are enabled in the detector config
(`/opt/ros/jazzy/share/apriltag_ros/cfg/tags_36h11.yaml`). An id missing there is
never detected and nothing warns you.

If a variable number of tags per row suits you better, use
`teleop/lambda_fr3_bridge_split_csv.py`, which writes tags long-format to a
second file joinable on `t`.

## 4.4 Which frame the tag poses are in

`lambda_fr3_bridge.py` looks the tags up **in the robot base frame** — the parent
frame for the TF lookup is hard-coded to `base`, so the `--camera-frame` flag has
no effect in that script. This means:

> `perception/calibration/camera_to_robot_tf.py` must be running for the whole
> session, or every tag column will be NaN.

That is a deliberate choice: it puts tag poses and end-effector poses in the same
frame so a recording is directly usable, at the cost of depending on the
calibration node. See [05_apriltag_calibration.md](05_apriltag_calibration.md).

`sync_logger.py` makes it explicit instead. `--tag-frame base` is the default and
behaves the same way; `--tag-frame ""` falls back to the detections header frame,
which is the camera optical frame, and needs no calibration node. Pick one per
project and write it down — a mix of base-frame and camera-frame files in the
same dataset is a painful thing to discover later.

## 4.5 Logging while teleoperating

The bridge starts a recording when you press `s` to START and closes it on STOP,
on loss of pose freshness, or on Ctrl+C. Writing happens on its own thread, so
it does not slow the 500 Hz control loop.

```bash
python3 teleop/lambda_fr3_bridge.py --task-type all_tasks --tag-ids 10 11 12 13 14
```

Files land in:

```
<--log-dir>/logs/<--task-type>/<task-type>_<YYYYmmdd_HHMMSS>_<NNN>.csv
```

for example `./logs/all_tasks/all_tasks_20260729_150004_001.csv`. The counter
increments per START within one bridge process, and the timestamp keeps files
unique across processes.

| Flag | Default | Meaning |
|---|---|---|
| `--task-type` | `default` | subfolder and filename prefix; use one per task |
| `--log-dir` | `.` | root under which `logs/<task-type>/` is created |
| `--log-rate` | 30 | rows per second |
| `--csv-log` / `--no-csv-log` | on | record at all |
| `--require-apriltag` / `--no-require-apriltag` | on | refuse to start unless `/detections` has a publisher |
| `--tag-ids` | 10 11 12 13 14 | tag column blocks to reserve |
| `--det-timeout` | 0.5 | oldest detection whose pose is still written, seconds |

To record a teleoperation session with no camera at all:

```bash
python3 teleop/lambda_fr3_bridge.py --task-type freespace --no-require-apriltag
```

Tag columns are still present and all NaN, so the schema does not change.

## 4.6 Logging without the bridge

`logging/sync_logger.py` records the same thing while something else moves the
arm — hand guidance, `move_to_start`, a scripted controller, or a bridge run you
started with `--no-csv-log`.

```bash
# FR3 + lambda.7 only
python3 logging/sync_logger.py --task-type handguided --rate 100 --no-tags

# FR3 + lambda.7 + tags, poses in the base frame
python3 logging/sync_logger.py --task-type pick_glass --rate 100 \
  --tags --tag-ids 10 11 12 --tag-frame base

# tags in the camera optical frame, no calibration node needed
python3 logging/sync_logger.py --task-type camtest --tags --tag-frame ""
```

Without `--tags` or `--no-tags` it decides on its own: tags on if `apriltag_msgs`
and `tf2_ros` import and `/detections` already has a publisher. Be explicit in
anything scripted.

Ctrl+C closes the file and prints the row count and path.

| Flag | Default | Meaning |
|---|---|---|
| `--rate` | 100 | rows per second |
| `--max-age` | 0.25 | a signal older than this is written NaN instead of held |
| `--task-type` | `default` | subfolder and filename prefix |
| `--log-dir` | `.` | root under which `logs/<task-type>/` is created |
| `--run` | 1 | run index in the filename |
| `--tags` / `--no-tags` | auto | log tag columns |
| `--tag-ids` | 10 11 12 13 14 | tag column blocks to reserve |
| `--tag-frame` | `base` | frame for tag lookups; `""` uses the camera optical frame |
| `--det-timeout` | 0.5 | oldest detection whose pose is still written, seconds |

`--max-age` is the one to think about. It must be comfortably above the period of
your slowest non-camera topic, or you will punch NaN holes in perfectly good
data. At 0.25 s it is 250 times the FR3 state period, which is deliberate: it is
there to catch a dead publisher, not jitter.

## 4.7 Check every recording

```bash
python3 logging/check_log.py logs/all_tasks/all_tasks_20260729_150004_001.csv
python3 logging/check_log.py logs/all_tasks/*.csv --gap 0.2
```

It needs pandas but no ROS environment, and reports the achieved rate, timing
gaps, per-signal coverage and per-tag visibility:

```
  rows            2566
  duration        87.5 s
  mean rate       29.3 Hz
  sample dt       median 34.0 ms, p99 35.9 ms, max 46.7 ms
  signal coverage (share of rows with a value)
    FR3 pose       100.0 %
    FR3 joints     100.0 %
    FR3 wrench     100.0 %
    gripper          0.0 %  <-- check this
    lambda pose    100.0 %
    lambda force   100.0 %
  AprilTag visibility
    tag 10         100.0 %
    tag 11         100.0 %
    tag 12          52.7 %
    tag 13          70.5 %
    tag 14          94.7 %
```

That is a real recording from this setup, and it shows both failure modes worth
catching. The gripper column is entirely empty — the topic name is wrong, see the
root README. Tag 12 was visible in barely half the frames, which is a camera
placement or occlusion problem, not a software one. Both are cheap to fix on the
day and expensive to discover weeks later.

Run it before you consider a session done.

## 4.8 Reading the data

```python
import pandas as pd

df = pd.read_csv("logs/all_tasks/all_tasks_20260729_150004_001.csv")
df["t"] -= df["t"].iloc[0]                      # seconds from session start

ee = df[["ee_x", "ee_y", "ee_z"]]
force = df[["fr3_Fx", "fr3_Fy", "fr3_Fz"]]
tag12 = df.filter(like="tag12_").dropna()       # only frames where tag 12 was seen

print(df["tag12_x"].isna().mean())               # fraction of rows without tag 12
print(df[["fd_Fx", "fd_Fy", "fd_Fz"]].abs().max())
```

For downstream learning, the useful convention is: drop rows where the robot
state is NaN (there should be none), and forward-fill or mask the tag columns
depending on whether your model can handle missing observations. Do not
forward-fill the force columns — a stale force reads as a real contact.

## 4.9 rosbag instead

```bash
python3 logging/record_rosbag.py -o demo_001               # curated topic set
python3 logging/record_rosbag.py -o demo_001 --all         # everything
python3 logging/record_rosbag.py -o demo_001 --storage mcap
ros2 bag info demo_001
```

Bags keep original message timestamps and per-topic rates, so they are the better
choice if you need to reconstruct exact timing or replay through ROS. The
downside is that you then have to do the time alignment yourself, which is the
work the CSV path has already done. Recording both at once is fine — the CSV is
small compared to the bag.

## 4.10 What the recordings do not contain

- **Grasp force.** Commanded, not measured. See 4.2.
- **Wrist torques on the lambda.7.** No torque is ever sent to the wrist axes, so
  there is nothing to log.
- **Camera images.** Only tag poses. Record a bag alongside if you need frames.
- **Depth.** The AprilTag pipeline uses the colour stream only. Depth is
  streaming but unused.
- **Controller internals.** Stiffness, the commanded equilibrium pose and the
  controller's own state are not in the CSV. `equilibrium_pose` is in the curated
  rosbag topic set if you need it.
