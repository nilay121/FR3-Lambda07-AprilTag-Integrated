#!/usr/bin/env python3
"""
sync_logger.py -- standalone synchronised CSV logger for FR3 + lambda.7 (+ AprilTag).

The teleoperation bridge (teleop/lambda_fr3_bridge.py) already logs while it is
driving the arm. This node does the same job without the bridge, so you can
record any session: hand-guided demonstrations, move_to_start runs, gripper
tests, or a bridge run whose logging you disabled with --no-csv-log.

Two logging modes, selected with --tags / --no-tags:

  two-signal  FR3 + lambda.7 only. No camera needed.
  three-signal  FR3 + lambda.7 + AprilTag poses from the RealSense pipeline.

The CSV schema is identical in both modes and identical to the one the bridge
writes, so the same downstream code reads every file in this repo. In two-signal
mode the tag columns are simply all NaN.

Synchronisation model
---------------------
One timer fires at --rate. Every row is one sample of the whole system taken at
one instant: the latest value received on each topic at that moment. Nothing is
interpolated and nothing is resampled after the fact.

Because the source topics run at different rates (FR3 state ~1 kHz, lambda.7
~1 kHz, camera ~30 Hz), a row is a zero-order hold of each signal. Any signal
that has not been updated within --max-age seconds is written as NaN rather than
held forever, so a dead publisher shows up as a gap in the data instead of a
flat line that looks like real measurement. Set --max-age generously above the
period of your slowest topic; the camera path has its own --det-timeout.

Run:
    source /opt/ros/jazzy/setup.bash
    source ~/franka_ros2_ws/install/setup.bash
    source ~/lambda-07_ws/install/setup.bash
    python3 sync_logger.py --task-type handguided --rate 100 --no-tags

Note: only syntax-checked off-hardware. Verify the topic names on your machine
with `ros2 topic list` before trusting a recording.
"""

import argparse
import csv
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState


NAN = float("nan")

# The two franka_robot_state_broadcaster convenience topics publish BEST_EFFORT.
# A reliable subscriber receives nothing from them at all.
BEST_EFFORT = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)

# Same column layout as teleop/lambda_fr3_bridge.py.
STATE_HEADER = (
    ["t", "ee_x", "ee_y", "ee_z", "ee_qx", "ee_qy", "ee_qz", "ee_qw"]
    + [f"q{i + 1}" for i in range(7)]
    + [f"dq{i + 1}" for i in range(7)]
    + ["gripper_width", "gripper_closed_cmd"]
    + ["fd_x", "fd_y", "fd_z", "fd_qx", "fd_qy", "fd_qz", "fd_qw"]
    + ["fd_Fx", "fd_Fy", "fd_Fz"]
    + ["fr3_Fx", "fr3_Fy", "fr3_Fz"]
)

TAG_FIELDS = ["x", "y", "z", "qx", "qy", "qz", "qw", "decision_margin"]


def tag_columns(tag_ids):
    cols = []
    for tag_id in tag_ids:
        cols += [f"tag{tag_id}_{field}" for field in TAG_FIELDS]
    return cols


try:
    from apriltag_msgs.msg import AprilTagDetectionArray

    _HAVE_APRILTAG = True
except Exception:
    _HAVE_APRILTAG = False

try:
    from tf2_ros import Buffer, TransformListener

    _HAVE_TF2 = True
except Exception:
    _HAVE_TF2 = False


class SyncLogger(Node):
    def __init__(self, args):
        super().__init__("fr3_lambda7_sync_logger")
        self.a = args

        # Latest sample of each signal, with the time it arrived.
        self.fr_pose = None
        self.fr_quat = None
        self.fr_pose_t = 0.0
        self.wrench = None
        self.wrench_t = 0.0
        self.arm_js = None
        self.arm_js_t = 0.0
        self.grip_width = None
        self.grip_t = 0.0
        self.fd_pose = None
        self.fd_quat = None
        self.fd_pose_t = 0.0
        self.fd_force = None
        self.fd_force_t = 0.0

        self.latest_dets = None
        self.latest_dets_t = 0.0
        self._tag_cache = None
        self._tag_cache_t = None

        self.tf_buffer = None
        self.tf_listener = None

        self.rows_written = 0
        self._lock = threading.Lock()
        self._file = None
        self._csv = None

        self.create_subscription(
            PoseStamped, args.franka_pose_topic, self._cb_fr_pose, BEST_EFFORT
        )
        self.create_subscription(
            WrenchStamped, args.franka_wrench_topic, self._cb_wrench, BEST_EFFORT
        )
        self.create_subscription(
            JointState, args.joint_states_topic, self._cb_arm_js, 10
        )
        self.create_subscription(JointState, args.gripper_topic, self._cb_grip, 10)
        self.create_subscription(PoseStamped, args.fd_pose_topic, self._cb_fd_pose, 10)
        self.create_subscription(
            JointState, args.fd_joint_topic, self._cb_fd_joint, 10
        )

        self.tags_enabled = self._resolve_tag_mode()
        if self.tags_enabled:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(
                AprilTagDetectionArray, args.detections_topic, self._cb_dets, 10
            )

        self._open_file()
        self.create_timer(1.0 / args.rate, self._write_row)

    # ---------------------------------------------------------------- setup

    def _resolve_tag_mode(self):
        """Decide whether to log tag columns. --tags forces on, --no-tags off."""
        if self.a.tags is False:
            return False

        missing = not (_HAVE_APRILTAG and _HAVE_TF2)
        if missing:
            if self.a.tags is True:
                self.get_logger().error(
                    "--tags requested but apriltag_msgs / tf2_ros are not "
                    "importable. Install them or run with --no-tags."
                )
                raise SystemExit(1)
            self.get_logger().warn(
                "apriltag_msgs / tf2_ros unavailable; logging without tags."
            )
            return False

        publishing = False
        try:
            publishing = self.count_publishers(self.a.detections_topic) > 0
        except Exception:
            pass

        if not publishing:
            if self.a.tags is True:
                self.get_logger().warn(
                    f"no publisher on {self.a.detections_topic} yet; tag columns "
                    "will fill in once the AprilTag pipeline starts."
                )
                return True
            self.get_logger().info(
                f"no publisher on {self.a.detections_topic}; logging without tags."
            )
            return False

        return True

    def _open_file(self):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        log_dir = os.path.join(
            os.path.expanduser(self.a.log_dir), "logs", self.a.task_type
        )
        os.makedirs(log_dir, exist_ok=True)
        path = os.path.join(log_dir, f"{self.a.task_type}_{stamp}_{self.a.run:03d}.csv")

        self._file = open(path, "w", newline="")
        self._csv = csv.writer(self._file)
        self._csv.writerow(STATE_HEADER + tag_columns(self.a.tag_ids))
        self.path = path

        mode = "FR3 + lambda.7 + tags" if self.tags_enabled else "FR3 + lambda.7"
        self.get_logger().info(
            f"logging {mode} -> {path} at {self.a.rate:.0f} Hz "
            f"(max-age {self.a.max_age:.2f} s)"
        )

    # ------------------------------------------------------------ callbacks

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _cb_fr_pose(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        self.fr_pose = [p.x, p.y, p.z]
        self.fr_quat = [o.x, o.y, o.z, o.w]
        self.fr_pose_t = self._now()

    def _cb_wrench(self, msg):
        f = msg.wrench.force
        self.wrench = [f.x, f.y, f.z]
        self.wrench_t = self._now()

    def _cb_arm_js(self, msg):
        self.arm_js = msg
        self.arm_js_t = self._now()

    def _cb_grip(self, msg):
        if len(msg.position) >= 2:
            self.grip_width = msg.position[0] + msg.position[1]
            self.grip_t = self._now()

    def _cb_fd_pose(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        self.fd_pose = [p.x, p.y, p.z]
        self.fd_quat = [o.x, o.y, o.z, o.w]
        self.fd_pose_t = self._now()

    def _cb_fd_joint(self, msg):
        eff = list(msg.effort)
        if eff:
            self.fd_force = (eff + [NAN] * 3)[:3]
            self.fd_force_t = self._now()

    def _cb_dets(self, msg):
        self.latest_dets = msg
        self.latest_dets_t = self._now()

    # ---------------------------------------------------------------- rows

    def _fresh(self, stamp, t):
        """True if a signal last updated at `stamp` is still valid at time `t`."""
        return stamp > 0.0 and (t - stamp) <= self.a.max_age

    def _arm_field(self, field, n):
        js = self.arm_js
        if js is None:
            return [NAN] * n
        values = getattr(js, field)
        out = [v for nm, v in zip(js.name, values) if nm.startswith("fr3_joint")]
        if not out:
            out = list(values)
        return (out + [NAN] * n)[:n]

    def _tag_pose_map(self, t):
        """id -> [x,y,z,qx,qy,qz,qw,decision_margin] in --tag-frame.

        The TF lookups are the most expensive work per row, so they run at most
        once per detection message (camera rate) and the result is cached for
        the faster logging ticks in between.
        """
        if not self.tags_enabled or self.tf_buffer is None:
            return {}

        dets = self.latest_dets
        dets_t = self.latest_dets_t
        if dets is None or (t - dets_t) > self.a.det_timeout:
            return {}

        if self._tag_cache is not None and dets_t == self._tag_cache_t:
            return self._tag_cache

        parent = self.a.tag_frame or dets.header.frame_id
        if not parent:
            return {}

        wanted = set(self.a.tag_ids)
        poses = {}
        for det in dets.detections:
            if det.id not in wanted:
                continue
            child = f"{det.family}:{det.id}"
            try:
                tf = self.tf_buffer.lookup_transform(parent, child, Time())
            except Exception:
                continue
            tr = tf.transform.translation
            rot = tf.transform.rotation
            poses[det.id] = [
                tr.x, tr.y, tr.z,
                rot.x, rot.y, rot.z, rot.w,
                det.decision_margin,
            ]

        self._tag_cache = poses
        self._tag_cache_t = dets_t
        return poses

    def _write_row(self):
        t = self._now()

        if self._fresh(self.fr_pose_t, t) and self.fr_pose is not None:
            ee, eq = list(self.fr_pose), list(self.fr_quat)
        else:
            ee, eq = [NAN] * 3, [NAN] * 4

        if self._fresh(self.arm_js_t, t):
            q = self._arm_field("position", 7)
            dq = self._arm_field("velocity", 7)
        else:
            q, dq = [NAN] * 7, [NAN] * 7

        gw = self.grip_width if self._fresh(self.grip_t, t) else NAN

        if self._fresh(self.fd_pose_t, t) and self.fd_pose is not None:
            lp, lq = list(self.fd_pose), list(self.fd_quat)
        else:
            lp, lq = [NAN] * 3, [NAN] * 4

        lf = (
            list(self.fd_force)
            if self._fresh(self.fd_force_t, t) and self.fd_force is not None
            else [NAN] * 3
        )
        wr = (
            list(self.wrench)
            if self._fresh(self.wrench_t, t) and self.wrench is not None
            else [NAN] * 3
        )

        # gripper_closed_cmd is a bridge-side command; standalone logging has no
        # visibility of it. The column is kept so the schema stays identical.
        row = [t] + ee + eq + q + dq + [gw, NAN] + lp + lq + lf + wr

        tag_poses = self._tag_pose_map(t)
        for tag_id in self.a.tag_ids:
            row += tag_poses.get(tag_id, [NAN] * len(TAG_FIELDS))

        with self._lock:
            if self._csv is not None:
                self._csv.writerow(row)
                self.rows_written += 1

        if self.rows_written % int(max(1.0, self.a.rate) * 10) == 0:
            self.get_logger().info(f"{self.rows_written} rows")

    def close(self):
        with self._lock:
            try:
                if self._file is not None:
                    self._file.flush()
                    self._file.close()
            except Exception:
                pass
            self._file = None
            self._csv = None
        self.get_logger().info(f"wrote {self.rows_written} rows -> {self.path}")


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="Synchronised CSV logger for FR3 + lambda.7 (+ AprilTag)."
    )

    parser.add_argument("--rate", type=float, default=100.0, help="rows per second")
    parser.add_argument(
        "--max-age",
        type=float,
        default=0.25,
        help="a signal older than this is written as NaN instead of held",
    )
    parser.add_argument(
        "--task-type",
        default="default",
        help="task name; logs go to <log-dir>/logs/<task-type>/",
    )
    parser.add_argument("--log-dir", default=".", help="root directory for logs")
    parser.add_argument(
        "--run", type=int, default=1, help="run index appended to the filename"
    )

    parser.add_argument(
        "--tags",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="log AprilTag columns; default auto-detects a /detections publisher",
    )
    parser.add_argument(
        "--tag-ids",
        type=int,
        nargs="+",
        default=[10, 11, 12, 13, 14],
        help="tag ids to reserve a column block for; ids not listed are dropped",
    )
    parser.add_argument(
        "--tag-frame",
        default="base",
        help="frame the tag poses are expressed in. 'base' requires "
        "perception/calibration/camera_to_robot_tf.py to be running. Pass an "
        "empty string to use the detections header frame (camera optical).",
    )
    parser.add_argument(
        "--det-timeout",
        type=float,
        default=0.5,
        help="max age of an AprilTag detection whose pose is still logged",
    )

    parser.add_argument("--detections-topic", default="/detections")
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument("--gripper-topic", default="/fr3_gripper/joint_states")
    parser.add_argument("--fd-pose-topic", default="/fd/ee_pose")
    parser.add_argument("--fd-joint-topic", default="/fd/joint_states")
    parser.add_argument(
        "--franka-pose-topic",
        default="/franka_robot_state_broadcaster/current_pose",
    )
    parser.add_argument(
        "--franka-wrench-topic",
        default="/franka_robot_state_broadcaster/external_wrench_in_base_frame",
    )

    return parser


def main():
    args, _ = build_argument_parser().parse_known_args()

    rclpy.init(args=sys.argv)
    node = SyncLogger(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
