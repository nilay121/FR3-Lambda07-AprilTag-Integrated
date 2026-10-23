#!/usr/bin/env python3
"""
lambda_fr3_bridge.py -- ROS 2 Jazzy teleoperation bridge

Controls a Franka FR3 with a Force Dimension lambda.7 and reflects the
Franka external translational force back to the lambda.7.

Controls
--------
- Press lowercase "s" in the bridge terminal to toggle teleoperation START/STOP.
- The lambda clutch/gripper axis remains available for Franka Hand open/close.
- Ctrl+C commands zero force before shutdown.

Capabilities
------------
- Relative 3-DOF translation control.
- Optional relative 3-DOF orientation control.
- Lambda gripper angle -> Franka Hand grasp/move.
- Optional 3-axis translational force feedback.
- Automatic CSV logging (no rosbag): when teleoperation is enabled AND the
  RealSense AprilTag pipeline is publishing detections, the bridge records, on a
  dedicated thread, a time-stamped CSV of the FR3 end-effector pose, arm joint
  states, gripper state, lambda pose + force, and the camera-frame poses of every
  visible AprilTag. Logging stops when teleoperation stops or the node exits.

This Python node is intended for low-gain bring-up and testing. A production
bilateral teleoperation system should use a real-time controller with an
independent command watchdog and passivity protection.
"""

import argparse
import csv
import os
import select
import sys
import termios
import threading
import time
import tty

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray


BEST_EFFORT = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)

NAN = float("nan")

# One CSV, one row per logging tick. State first, then a fixed column block per
# configured tag id (wide format), so every signal shares the same `t`.
STATE_HEADER = (
    ["t", "ee_x", "ee_y", "ee_z", "ee_qx", "ee_qy", "ee_qz", "ee_qw"]
    + [f"q{i + 1}" for i in range(7)]
    + [f"dq{i + 1}" for i in range(7)]
    + ["gripper_width", "gripper_closed_cmd"]
    + ["fd_x", "fd_y", "fd_z", "fd_qx", "fd_qy", "fd_qz", "fd_qw"]
    + ["fd_Fx", "fd_Fy", "fd_Fz"]
    + ["fr3_Fx", "fr3_Fy", "fr3_Fz"]
)

# Per-tag fields (the tag id is encoded in the column-name prefix). Poses are in
# the CAMERA optical frame; convert to the robot base frame downstream.
TAG_FIELDS = ["x", "y", "z", "qx", "qy", "qz", "qw", "decision_margin"]


def tag_columns(tag_ids):
    """Wide-format tag column names, e.g. tag5_x .. tag5_decision_margin."""
    cols = []
    for tag_id in tag_ids:
        cols += [f"tag{tag_id}_{field}" for field in TAG_FIELDS]
    return cols

try:
    from franka_msgs.action import Grasp, Move

    _HAVE_GRIPPER = True
except Exception:
    _HAVE_GRIPPER = False

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


class Bridge(Node):
    def __init__(self, args):
        super().__init__("lambda_fr3_bridge")
        self.a = args

        # Lambda-to-Franka axis mapping. Keep identity until the physical axis
        # correspondence has been verified one axis at a time.
        self.Rmap = np.eye(3)

        # Device state caches.
        self.fd_pose = None
        self.fd_quat = None
        self.fd_grip = None
        self.button = False  # retained for diagnostics; not used as deadman

        self.fr_pose = None
        self.fr_quat = None
        self.wrench = None
        self.fd_force = None      # lambda rendered force (N) from /fd/joint_states effort

        # Logging-only state caches (not used by the control loop).
        self.arm_js = None        # /joint_states (sensor_msgs/JointState)
        self.grip_width = None    # Franka Hand width (m)
        self.latest_dets = None   # AprilTagDetectionArray
        self.latest_dets_t = 0.0

        # Message freshness timestamps.
        self.last_fd_t = 0.0
        self.last_fr_t = 0.0
        self.last_wrench_t = 0.0

        # Keyboard teleoperation state.
        self.keyboard_enabled = False
        self._keyboard_stop = threading.Event()
        self._state_lock = threading.Lock()
        self._shutdown_started = False

        # Relative-motion engagement state.
        self.engaged = False
        self.pL0 = None
        self.pF0 = None
        self.qL0 = None
        self.qF0 = None
        self.target = None
        self.target_quat = None

        # Franka external-wrench bias.
        self.bias = np.zeros(3)
        self._bias_buf = []
        self.bias_ready = False

        # Gripper state.
        self.gripper_closed = False

        # CSV logging state (dedicated thread; isolated from the control loop).
        self._log_lock = threading.Lock()
        self._log_thread = None
        self._log_stop = threading.Event()
        self._log_file = None
        self._log_csv = None
        self._log_counter = 0
        # Tag-pose cache: only re-run TF lookups when a NEW detection arrives,
        # so the 100 Hz logger does not hammer TF at the camera can't-keep-up rate.
        self._tag_cache = None
        self._tag_cache_t = None

        # Publishers and subscribers.
        self.pub_eq = self.create_publisher(
            PoseStamped,
            args.equilibrium_topic,
            10,
        )
        self.pub_force = self.create_publisher(
            Float64MultiArray,
            args.fd_command_topic,
            10,
        )

        self.create_subscription(
            PoseStamped,
            args.fd_pose_topic,
            self._cb_fd_pose,
            10,
        )
        self.create_subscription(
            Bool,
            args.fd_button_topic,
            self._cb_button,
            10,
        )
        self.create_subscription(
            JointState,
            args.fd_joint_topic,
            self._cb_fd_joint,
            10,
        )
        self.create_subscription(
            PoseStamped,
            args.franka_pose_topic,
            self._cb_fr_pose,
            BEST_EFFORT,
        )
        self.create_subscription(
            WrenchStamped,
            args.franka_wrench_topic,
            self._cb_wrench,
            BEST_EFFORT,
        )

        # Logging-only subscriptions.
        self.create_subscription(
            JointState,
            args.joint_states_topic,
            self._cb_arm_js,
            10,
        )
        self.create_subscription(
            JointState,
            args.gripper_topic,
            self._cb_grip,
            10,
        )
        if _HAVE_APRILTAG:
            self.create_subscription(
                AprilTagDetectionArray,
                args.detections_topic,
                self._cb_dets,
                10,
            )

        # TF listener for AprilTag poses (camera_frame -> <family>:<id>).
        # spin_thread=True runs the /tf + /tf_static processing on the listener's
        # OWN executor thread, so the high-rate Franka TF tree is not handled on
        # the same executor as the 500 Hz control loop (avoids motion jitter).
        self.tf_buffer = None
        self.tf_listener = None
        if _HAVE_TF2:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(
                self.tf_buffer, self, spin_thread=True
            )

        # Optional Franka Hand actions.
        self.grasp_cli = None
        self.move_cli = None
        if _HAVE_GRIPPER:
            self.grasp_cli = ActionClient(self, Grasp, args.grasp_action)
            self.move_cli = ActionClient(self, Move, args.move_action)
        else:
            self.get_logger().warn(
                "franka_msgs actions are unavailable; gripper control is disabled."
            )

        # Control timers.
        self.create_timer(1.0 / args.rate, self._tick)
        self.create_timer(1.0 / 15.0, self._gripper_tick)

        # Keyboard listener.
        self._keyboard_thread = threading.Thread(
            target=self._keyboard_loop,
            name="lambda_fr3_keyboard",
            daemon=True,
        )
        self._keyboard_thread.start()

        # Best-effort normal-shutdown safety hook.
        rclpy.get_default_context().on_shutdown(self._on_shutdown)

        self.get_logger().warn(
            "Press 's' to START/STOP teleoperation. The lambda clutch remains "
            "mapped to the Franka gripper. Force is zero while stopped."
        )
        self.get_logger().warn(
            "Rmap is identity. Verify force and motion direction one axis at a time."
        )

    # ------------------------------------------------------------------
    # Time and callbacks
    # ------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _cb_fd_pose(self, msg):
        self.fd_pose = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ],
            dtype=float,
        )
        self.last_fd_t = self._now()

    def _cb_button(self, msg):
        # Stored only for diagnostics. It is not used to engage teleoperation.
        self.button = bool(msg.data)

    def _cb_fd_joint(self, msg):
        positions = {
            name: (
                msg.position[index]
                if index < len(msg.position)
                else float("nan")
            )
            for index, name in enumerate(msg.name)
        }

        grip = positions.get(self.a.gripper_joint)
        if grip is not None and np.isfinite(grip):
            self.fd_grip = float(grip)

        # Lambda rendered force (device effort on the translational axes), logged
        # as the lambda-side force. Not used by the control loop.
        efforts = {
            name: (msg.effort[index] if index < len(msg.effort) else NAN)
            for index, name in enumerate(msg.name)
        }
        fxyz = [efforts.get("fd_x"), efforts.get("fd_y"), efforts.get("fd_z")]
        if all(v is not None for v in fxyz):
            force = np.array(fxyz, dtype=float)
            if np.all(np.isfinite(force)):
                self.fd_force = force

        roll = positions.get("fd_roll")
        pitch = positions.get("fd_pitch")
        yaw = positions.get("fd_yaw")

        if roll is None or pitch is None or yaw is None:
            return

        rpy = np.array([roll, pitch, yaw], dtype=float)
        if not np.all(np.isfinite(rpy)):
            return

        self.fd_quat = Rotation.from_euler(
            "xyz",
            rpy,
            degrees=False,
        ).as_quat()

    def _cb_fr_pose(self, msg):
        self.fr_pose = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ],
            dtype=float,
        )

        orientation = msg.pose.orientation
        quat = np.array(
            [orientation.x, orientation.y, orientation.z, orientation.w],
            dtype=float,
        )
        if np.all(np.isfinite(quat)) and np.linalg.norm(quat) > 1e-9:
            self.fr_quat = quat / np.linalg.norm(quat)

        self.last_fr_t = self._now()

    def _cb_wrench(self, msg):
        force = msg.wrench.force
        wrench = np.array([force.x, force.y, force.z], dtype=float)
        if np.all(np.isfinite(wrench)):
            self.wrench = wrench
            self.last_wrench_t = self._now()

    def _cb_arm_js(self, msg):
        self.arm_js = msg

    def _cb_grip(self, msg):
        if len(msg.position) >= 2:
            self.grip_width = float(msg.position[0] + msg.position[1])
        elif msg.position:
            self.grip_width = float(msg.position[0])

    def _cb_dets(self, msg):
        self.latest_dets = msg
        self.latest_dets_t = self._now()

    # ------------------------------------------------------------------
    # Keyboard control
    # ------------------------------------------------------------------
    def _keyboard_loop(self):
        if not sys.stdin.isatty():
            self.get_logger().error(
                "Keyboard control is unavailable because stdin is not a terminal."
            )
            return

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)

            while not self._keyboard_stop.is_set() and rclpy.ok():
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not readable:
                    continue

                key = sys.stdin.read(1).lower()
                if key != "s":
                    continue

                with self._state_lock:
                    enable = not self.keyboard_enabled

                if enable:
                    self._enable_teleoperation()
                else:
                    self._disable_teleoperation(
                        "STOP: keyboard teleoperation disabled",
                        repeat_zero=True,
                    )

        except Exception as exc:
            self.get_logger().error(f"Keyboard listener failed: {exc}")
            self._disable_teleoperation(
                "STOP: keyboard listener failure",
                repeat_zero=True,
            )
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

    def _enable_teleoperation(self):
        with self._state_lock:
            self.keyboard_enabled = True
            self.engaged = False
            self.pL0 = None
            self.pF0 = None
            self.qL0 = None
            self.qF0 = None

        self.get_logger().warn(
            "START: teleoperation enabled; origins will latch on the next fresh cycle"
        )

        # Start CSV logging alongside teleoperation.
        self._start_logging()

    def _disable_teleoperation(self, reason, repeat_zero=False):
        with self._state_lock:
            was_enabled = self.keyboard_enabled or self.engaged
            self.keyboard_enabled = False
            self.engaged = False
            self.pL0 = None
            self.pF0 = None
            self.qL0 = None
            self.qF0 = None

        if repeat_zero:
            self._publish_zero_force_repeatedly(count=10)
        else:
            self._publish_force(np.zeros(3))

        # Finalise the CSV whenever teleoperation stops.
        self._stop_logging()

        if was_enabled:
            self.get_logger().warn(reason)

    # ------------------------------------------------------------------
    # Main bilateral loop
    # ------------------------------------------------------------------
    def _tick(self):
        now = self._now()

        fd_fresh = (
            self.fd_pose is not None
            and (now - self.last_fd_t) < self.a.watchdog
        )
        fr_fresh = (
            self.fr_pose is not None
            and self.fr_quat is not None
            and (now - self.last_fr_t) < self.a.watchdog
        )
        wrench_fresh = (
            self.wrench is not None
            and (now - self.last_wrench_t) < self.a.wrench_watchdog
        )

        with self._state_lock:
            keyboard_enabled = self.keyboard_enabled

        # Collect unloaded Franka wrench bias while teleoperation is stopped.
        if (
            not keyboard_enabled
            and not self.bias_ready
            and wrench_fresh
            and np.all(np.isfinite(self.wrench))
        ):
            self._bias_buf.append(self.wrench.copy())

            if len(self._bias_buf) >= self.a.bias_samples:
                samples = np.asarray(self._bias_buf, dtype=float)
                self.bias = np.mean(samples, axis=0)
                self.bias_ready = True
                self.get_logger().info(
                    f"wrench bias ready: {np.round(self.bias, 3)} N"
                )

        # STOP state: hold the last Franka target and continuously command zero force.
        if not keyboard_enabled:
            self.engaged = False
            self.pL0 = None
            self.qL0 = None
            self._publish_force(np.zeros(3))

            if self.target is not None and self.target_quat is not None:
                self._publish_target(self.target, self.target_quat)
            return

        # Pose freshness is mandatory for motion. Stale pose data automatically stops.
        if not (fd_fresh and fr_fresh):
            self._disable_teleoperation(
                "STOP: stale Lambda or Franka pose data",
                repeat_zero=False,
            )
            if self.target is not None and self.target_quat is not None:
                self._publish_target(self.target, self.target_quat)
            return

        # First fresh cycle after START: latch relative origins.
        if not self.engaged:
            self.engaged = True
            self.pL0 = self.fd_pose.copy()
            self.pF0 = self.fr_pose.copy()
            self.qL0 = self.fd_quat.copy() if self.fd_quat is not None else None
            self.qF0 = self.fr_quat.copy()
            self.target = self.pF0.copy()
            self.target_quat = self.qF0.copy()
            self.get_logger().info("ENGAGED: Lambda and Franka origins latched")

        # Relative translation mapping.
        delta_lambda = self.fd_pose - self.pL0
        desired = self.pF0 + self.a.scale * (self.Rmap @ delta_lambda)

        # Workspace sphere around the engagement position.
        offset = desired - self.pF0
        radius = np.linalg.norm(offset)
        if radius > self.a.workspace and radius > 1e-12:
            desired = self.pF0 + offset * (self.a.workspace / radius)

        # Per-cycle target movement limit.
        step = desired - self.target
        step_norm = np.linalg.norm(step)
        if step_norm > self.a.max_step and step_norm > 1e-12:
            desired = self.target + step * (self.a.max_step / step_norm)

        self.target = desired
        self.target_quat = self._orientation_target()
        self._publish_target(self.target, self.target_quat)

        # Force feedback safety gates.
        if (
            not self.a.force_enabled
            or not self.bias_ready
            or not wrench_fresh
            or not np.all(np.isfinite(self.wrench))
        ):
            self._publish_force(np.zeros(3))
            return

        # Bias-corrected Franka force in the Franka base frame.
        franka_force = self.wrench - self.bias

        # Map into the Lambda frame and reflect in the opposing direction.
        lambda_force = -self.a.force_gain * (self.Rmap.T @ franka_force)

        # Axis-wise deadband.
        for axis in range(3):
            if abs(lambda_force[axis]) < self.a.deadband:
                lambda_force[axis] = 0.0

        # Vector magnitude limit.
        magnitude = np.linalg.norm(lambda_force)
        if magnitude > self.a.max_force and magnitude > 1e-12:
            lambda_force *= self.a.max_force / magnitude

        self._publish_force(lambda_force)

    def _orientation_target(self):
        if (
            not self.a.orientation_enabled
            or self.qL0 is None
            or self.fd_quat is None
        ):
            return self.qF0

        try:
            lambda_delta = (
                Rotation.from_quat(self.qL0).inv()
                * Rotation.from_quat(self.fd_quat)
            )

            rotvec = lambda_delta.as_rotvec() * self.a.rotation_scale
            angle = np.linalg.norm(rotvec)
            if angle > self.a.max_angle and angle > 1e-12:
                rotvec *= self.a.max_angle / angle

            return (
                Rotation.from_quat(self.qF0)
                * Rotation.from_rotvec(rotvec)
            ).as_quat()
        except ValueError as exc:
            self.get_logger().warn(f"Invalid orientation sample: {exc}")
            return self.qF0

    # ------------------------------------------------------------------
    # Publishers and shutdown safety
    # ------------------------------------------------------------------
    def _publish_target(self, position, quaternion):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.a.base_frame

        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])

        if quaternion is not None:
            msg.pose.orientation.x = float(quaternion[0])
            msg.pose.orientation.y = float(quaternion[1])
            msg.pose.orientation.z = float(quaternion[2])
            msg.pose.orientation.w = float(quaternion[3])
        else:
            msg.pose.orientation.w = 1.0

        self.pub_eq.publish(msg)

    def _publish_force(self, force_xyz):
        force_xyz = np.asarray(force_xyz, dtype=float)
        if force_xyz.shape != (3,) or not np.all(np.isfinite(force_xyz)):
            force_xyz = np.zeros(3)

        data = list(map(float, force_xyz))
        data.extend([0.0] * max(0, self.a.force_dim - 3))
        self.pub_force.publish(
            Float64MultiArray(data=data[: self.a.force_dim])
        )

    def _publish_zero_force_repeatedly(self, count=10):
        msg = Float64MultiArray(data=[0.0] * self.a.force_dim)
        for _ in range(max(1, count)):
            try:
                self.pub_force.publish(msg)
            except Exception:
                break
            time.sleep(0.002)

    def _on_shutdown(self):
        if self._shutdown_started:
            return

        self._shutdown_started = True
        self._keyboard_stop.set()

        with self._state_lock:
            self.keyboard_enabled = False
            self.engaged = False

        try:
            self.get_logger().warn("Shutdown: commanding zero Lambda force")
            self._publish_zero_force_repeatedly(count=20)
        except Exception:
            pass

        # Finalise any in-progress CSV so it is not left open.
        self._stop_logging()

    # ------------------------------------------------------------------
    # Franka Hand control
    # ------------------------------------------------------------------
    def _gripper_tick(self):
        if self.fd_grip is None or not _HAVE_GRIPPER:
            return

        angle = self.fd_grip

        if not self.gripper_closed and angle < self.a.grip_close_th:
            if self._send_grasp():
                self.gripper_closed = True
        elif self.gripper_closed and angle > self.a.grip_open_th:
            if self._send_move(self.a.grip_open_width):
                self.gripper_closed = False

    def _send_grasp(self):
        if self.grasp_cli is None:
            return False

        if not self.grasp_cli.server_is_ready():
            self.grasp_cli.wait_for_server(timeout_sec=0.0)
            if not self.grasp_cli.server_is_ready():
                return False

        goal = Grasp.Goal()
        goal.width = 0.0
        goal.speed = self.a.grip_speed
        goal.force = self.a.grip_force
        goal.epsilon.inner = 0.05
        goal.epsilon.outer = 0.05

        self.grasp_cli.send_goal_async(goal)
        self.get_logger().info("gripper: GRASP")
        return True

    def _send_move(self, width):
        if self.move_cli is None:
            return False

        if not self.move_cli.server_is_ready():
            self.move_cli.wait_for_server(timeout_sec=0.0)
            if not self.move_cli.server_is_ready():
                return False

        goal = Move.Goal()
        goal.width = float(width)
        goal.speed = self.a.grip_speed

        self.move_cli.send_goal_async(goal)
        self.get_logger().info("gripper: OPEN")
        return True

    # ------------------------------------------------------------------
    # CSV logging (dedicated thread; no rosbag)
    # ------------------------------------------------------------------
    def _apriltag_active(self):
        """True if the RealSense AprilTag pipeline is publishing /detections."""
        try:
            return self.count_publishers(self.a.detections_topic) > 0
        except Exception:
            return False

    def _start_logging(self):
        """Open the CSV and start the logging thread, once per teleop START.

        Gated on the AprilTag pipeline being live so every session has tag poses.
        """
        if not self.a.csv_log:
            return

        with self._log_lock:
            if self._log_thread is not None and self._log_thread.is_alive():
                return

            if not (_HAVE_APRILTAG and _HAVE_TF2):
                self.get_logger().warn(
                    "CSV logging: apriltag_msgs/tf2_ros unavailable; "
                    "tag columns will be empty."
                )

            if not self._apriltag_active():
                if self.a.require_apriltag:
                    self.get_logger().warn(
                        f"CSV logging: no publisher on {self.a.detections_topic}; "
                        "is the RealSense AprilTag pipeline running? Not logging "
                        "(pass --no-require-apriltag to log without tags)."
                    )
                    return
                self.get_logger().warn(
                    f"CSV logging: {self.a.detections_topic} has no publisher; "
                    "logging with empty tag columns."
                )

            self._log_counter += 1
            stamp = time.strftime("%Y%m%d_%H%M%S")
            # log_dir = os.path.expanduser(self.a.log_dir)

            log_dir = os.path.join(
                os.path.expanduser(self.a.log_dir),
                "logs",
                self.a.task_type,
            )

            # path = os.path.join(
            #     log_dir, f"{self.a.log_prefix}_{stamp}_{self._log_counter:02d}.csv"
            # )

            filename = (
                f"{self.a.task_type}_"
                f"{stamp}_"
                f"{self._log_counter:03d}.csv"
            )

            path = os.path.join(log_dir, filename)

            try:
                os.makedirs(log_dir, exist_ok=True)
                self._log_file = open(path, "w", newline="")
            except Exception as exc:
                self.get_logger().error(f"CSV logging: cannot open file: {exc}")
                self._close_files()
                return

            self._log_csv = csv.writer(self._log_file)
            self._log_csv.writerow(STATE_HEADER + tag_columns(self.a.tag_ids))

            self._log_stop.clear()
            self._log_thread = threading.Thread(
                target=self._logging_loop,
                name="lambda_fr3_csvlog",
                daemon=True,
            )
            self._log_thread.start()
            self.get_logger().warn(
                f"CSV logging -> {path} at {self.a.log_rate:.0f} Hz "
                f"(tag ids {list(self.a.tag_ids)})"
            )

    def _logging_loop(self):
        period = 1.0 / self.a.log_rate
        try:
            while not self._log_stop.is_set() and rclpy.ok():
                self._write_row()
                time.sleep(period)
        except Exception as exc:
            self.get_logger().error(f"CSV logging loop failed: {exc}")
        finally:
            with self._log_lock:
                self._close_files()

    def _arm_field(self, field, n):
        js = self.arm_js
        if js is None:
            return [NAN] * n
        values = getattr(js, field)
        out = [v for nm, v in zip(js.name, values) if nm.startswith("fr3_joint")]
        if not out:
            out = list(values)
        return (out + [NAN] * n)[:n]

    def _write_row(self):
        t = self._now()

        ee = list(self.fr_pose) if self.fr_pose is not None else [NAN] * 3
        eq = list(self.fr_quat) if self.fr_quat is not None else [NAN] * 4
        q = self._arm_field("position", 7)
        dq = self._arm_field("velocity", 7)
        gw = self.grip_width if self.grip_width is not None else NAN
        gc = 1 if self.gripper_closed else 0
        lp = list(self.fd_pose) if self.fd_pose is not None else [NAN] * 3
        lq = list(self.fd_quat) if self.fd_quat is not None else [NAN] * 4
        lf = list(self.fd_force) if self.fd_force is not None else [NAN] * 3
        wr = list(self.wrench) if self.wrench is not None else [NAN] * 3

        row = [t] + ee + eq + q + dq + [gw, gc] + lp + lq + lf + wr

        # Append one fixed block per configured tag id (NaN when not visible).
        tag_poses = self._tag_pose_map(t)
        for tag_id in self.a.tag_ids:
            row += tag_poses.get(tag_id, [NAN] * len(TAG_FIELDS))

        with self._log_lock:
            if self._log_csv is not None:
                self._log_csv.writerow(row)

    def _tag_pose_map(self, t):
        """id -> [x,y,z,qx,qy,qz,qw,decision_margin] in the camera frame.

        TF lookups are the heaviest per-tick work and hold the GIL, so they are
        run at most once per detection message (~camera rate) and cached; the
        100 Hz logger reuses the cache in between.
        """
        if not (_HAVE_APRILTAG and _HAVE_TF2) or self.tf_buffer is None:
            return {}

        dets = self.latest_dets
        dets_t = self.latest_dets_t
        if dets is None or (t - dets_t) > self.a.det_timeout:
            return {}

        # No new detection since the last lookup -> reuse the cached poses.
        if self._tag_cache is not None and dets_t == self._tag_cache_t:
            return self._tag_cache

        #parent = self.a.camera_frame or dets.header.frame_id
        parent = "base"
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

    def _stop_logging(self):
        self._log_stop.set()

        thread = self._log_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)

        with self._log_lock:
            self._log_thread = None
            self._close_files()

    def _close_files(self):
        try:
            if self._log_file is not None:
                self._log_file.flush()
                self._log_file.close()
        except Exception:
            pass
        self._log_file = None
        self._log_csv = None


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description=(
            "lambda.7 <-> FR3 bridge: keyboard-toggle teleoperation, "
            "6-DOF motion, gripper control and 3-DOF force feedback"
        )
    )

    # Rates and mapping.
    parser.add_argument("--rate", type=float, default=500.0)
    parser.add_argument("--scale", type=float, default=0.25)
    parser.add_argument(
        "--workspace",
        type=float,
        default=0.10,
        help="workspace radius in metres around the engagement pose",
    )
    parser.add_argument(
        "--max-step",
        type=float,
        default=5e-4,
        help="maximum Cartesian target step per bridge cycle in metres",
    )
    parser.add_argument(
        "--watchdog",
        type=float,
        default=0.10,
        help="Lambda and Franka pose freshness timeout in seconds",
    )
    parser.add_argument(
        "--wrench-watchdog",
        type=float,
        default=0.10,
        help="Franka wrench freshness timeout in seconds",
    )
    parser.add_argument("--base-frame", default="fr3_link0")

    # Orientation.
    parser.add_argument("--orientation-enabled", action="store_true")
    parser.add_argument("--rotation-scale", type=float, default=0.5)
    parser.add_argument(
        "--max-angle",
        type=float,
        default=0.35,
        help="maximum relative orientation angle in radians",
    )

    # Force feedback.
    parser.add_argument("--force-enabled", action="store_true")
    parser.add_argument("--force-gain", type=float, default=0.05)
    parser.add_argument("--max-force", type=float, default=0.8)
    parser.add_argument("--deadband", type=float, default=0.05)
    parser.add_argument(
        "--force-dim",
        type=int,
        choices=[3],
        default=3,
        help="Lambda effort controller command dimension; this setup uses 3",
    )
    parser.add_argument(
        "--bias-samples",
        type=int,
        default=500,
        help="number of unloaded Franka wrench samples used for bias",
    )

    # CSV logging (no rosbag).
    parser.add_argument(
        "--csv-log",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="record a CSV while teleoperation is enabled",
    )
    parser.add_argument(
        "--require-apriltag",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="only start logging when /detections has a publisher",
    )
    parser.add_argument(
        "--log-dir",
        default=".",
        help="directory for CSV output",
    )
    parser.add_argument(
        "--log-prefix",
        default="teleop",
        help="filename prefix for CSV output (timestamp appended)",
    )
    parser.add_argument(
        "--log-rate",
        type=float,
        default=30.0, #100.0,
        help="CSV sampling rate in Hz",
    )
    parser.add_argument(
        "--det-timeout",
        type=float,
        default=0.5,
        help="max age of an AprilTag detection to still log its pose, seconds",
    )
    parser.add_argument(
        "--tag-ids",
        type=int,
        nargs="+",
        default=[10, 11, 12, 13, 14],
        help="AprilTag ids to reserve a column block for (wide format). Set this "
        "to the tag ids in YOUR scene; ids not listed are not logged.",
    )
    parser.add_argument(
        "--detections-topic",
        default="/detections",
        help="AprilTag detections topic (apriltag_msgs/AprilTagDetectionArray)",
    )
    parser.add_argument(
        "--joint-states-topic",
        default="/joint_states",
        help="FR3 arm joint states topic",
    )
    parser.add_argument(
        "--gripper-topic",
        default="/fr3_gripper/joint_states",
        help="Franka Hand joint states topic (finger widths)",
    )
    parser.add_argument(
        "--camera-frame",
        default="",
        help="camera optical frame for tag-pose lookup; empty uses the "
        "detections message header frame_id",
    )

    parser.add_argument(
        "--task-type",
        default="default",
        help="Task name used to organize logs (e.g. pick_glass, stack_blocks)",
    )

    # Gripper.
    parser.add_argument("--gripper-joint", default="fd_clutch")
    parser.add_argument("--grip-close-th", type=float, default=0.03)
    parser.add_argument("--grip-open-th", type=float, default=0.06)
    parser.add_argument("--grip-open-width", type=float, default=0.08)
    parser.add_argument("--grip-speed", type=float, default=0.05)
    parser.add_argument("--grip-force", type=float, default=20.0)

    # Topics and actions.
    parser.add_argument("--fd-pose-topic", default="/fd/ee_pose")
    parser.add_argument("--fd-button-topic", default="/fd/button_state")
    parser.add_argument("--fd-joint-topic", default="/fd/joint_states")
    parser.add_argument(
        "--fd-command-topic",
        default="/fd/fd_controller/commands",
    )
    parser.add_argument(
        "--franka-pose-topic",
        default="/franka_robot_state_broadcaster/current_pose",
    )
    parser.add_argument(
        "--franka-wrench-topic",
        default=(
            "/franka_robot_state_broadcaster/"
            "external_wrench_in_base_frame"
        ),
    )
    parser.add_argument(
        "--equilibrium-topic",
        default=(
            "/cartesian_impedance_example_controller/"
            "equilibrium_pose"
        ),
    )
    parser.add_argument("--grasp-action", default="/franka_gripper/grasp")
    parser.add_argument("--move-action", default="/franka_gripper/move")

    return parser


def main():
    parser = build_argument_parser()
    args, ros_unknown = parser.parse_known_args()

    if args.bias_samples <= 0:
        parser.error("--bias-samples must be greater than zero")
    if args.rate <= 0.0:
        parser.error("--rate must be greater than zero")
    if args.log_rate <= 0.0:
        parser.error("--log-rate must be greater than zero")
    if len(set(args.tag_ids)) != len(args.tag_ids):
        parser.error("--tag-ids must not contain duplicates")
    if args.max_force < 0.0:
        parser.error("--max-force must be non-negative")
    if args.deadband < 0.0:
        parser.error("--deadband must be non-negative")

    rclpy.init(args=[sys.argv[0], *ros_unknown])
    node = Bridge(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._on_shutdown()

        # Give DDS a brief opportunity to send the zero-force commands.
        if rclpy.ok():
            for _ in range(10):
                try:
                    rclpy.spin_once(node, timeout_sec=0.01)
                except Exception:
                    break

        if node._keyboard_thread.is_alive():
            node._keyboard_thread.join(timeout=0.3)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()