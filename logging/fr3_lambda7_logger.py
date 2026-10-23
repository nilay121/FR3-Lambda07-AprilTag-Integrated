#!/usr/bin/env python3
"""
fr3_lambda7_logger.py  --  ROS 2 (rclpy) node  [Jazzy + franka_ros2 v3.4.1 + forcedimension_ros2]

Logs, to one time-synchronised CSV, using the topics the CURRENT stack actually
publishes (verify names with `ros2 topic list`):

  FR3 (franka_robot_state_broadcaster convenience topics -- BEST-EFFORT QoS):
    - EE position (x,y,z, m)            <- /franka_robot_state_broadcaster/current_pose
    - EE linear velocity (finite diff of current_pose)
    - FR3 external wrench (Fx..Tz)      <- /franka_robot_state_broadcaster/external_wrench_in_base_frame
  Arm joints (sensor_msgs/JointState):
    - q1..q7 (rad), dq1..dq7 (rad/s)    <- /joint_states  (fr3_joint* filtered)
  Franka Hand (sensor_msgs/JointState):
    - gripper width (m) = finger1+finger2 <- /fr3_gripper/joint_states
  lambda.7 (forcedimension_ros2, sensor_msgs/JointState):
    - rendered force per axis           <- /fd/joint_states  (EFFORT of fd_x/fd_y/fd_z[/roll/pitch/yaw])
    - gripper angle + gripper force      <- /fd/joint_states  (fd_clutch position + effort)

IMPORTANT QoS: the two franka convenience topics are BEST-EFFORT; this node
subscribes to them with best-effort QoS (a reliable subscriber would receive
nothing). Joint-state topics use default (reliable) QoS.

Lambda force source: default reads /fd/joint_states effort (forcedimension_ros2). If
you use the DHD fallback publisher instead, pass
  --lambda-source wrench --lambda-wrench-topic /lambda7/wrench

Run:  python3 fr3_lambda7_logger.py --out demo_001.csv --rate 100
"""
import argparse
import csv
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray

NAN = float("nan")
BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)


def _by_name(js, prefix, field="position", n=None):
    """Pull JointState.<field> for joints whose name starts with `prefix`, in msg order."""
    vals = getattr(js, field)
    out = [v for nm, v in zip(js.name, vals) if nm.startswith(prefix)]
    if not out:                                   # no name match -> take all (best effort)
        out = list(vals)
    return out if n is None else (out + [NAN] * n)[:n]


def _joint_effort_map(js):
    """name -> effort dict (for fd_clutch lookup etc.)."""
    return {nm: (js.effort[i] if i < len(js.effort) else NAN) for i, nm in enumerate(js.name)}


class Logger(Node):
    def __init__(self, args):
        super().__init__("fr3_lambda7_logger")
        self.args = args
        self.pose = None          # franka current_pose (PoseStamped)
        self.wr = None            # franka external wrench (WrenchStamped)
        self.js = None            # arm /joint_states
        self.grip = None          # /fr3_gripper/joint_states
        self.fd = None            # /fd/joint_states
        self.l7w = None           # optional lambda WrenchStamped (fallback)
        self._prev_p = None       # (t, x, y, z) for EE velocity finite-diff

        self.f = open(os.path.expanduser(args.out), "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(
            ["t", "ee_x", "ee_y", "ee_z", "ee_vx", "ee_vy", "ee_vz"]
            + [f"q{i+1}" for i in range(7)] + [f"dq{i+1}" for i in range(7)]
            + ["gripper_width"]
            + ["fr3_Fx", "fr3_Fy", "fr3_Fz", "fr3_Tx", "fr3_Ty", "fr3_Tz"]
            + ["l7_Fx", "l7_Fy", "l7_Fz", "l7_Tx", "l7_Ty", "l7_Tz",
               "l7_gripper_angle", "l7_gripper_force"]
        )

        self.create_subscription(PoseStamped, args.franka_pose_topic, self._cb_pose, BEST_EFFORT)
        self.create_subscription(WrenchStamped, args.franka_wrench_topic, self._cb_wr, BEST_EFFORT)
        self.create_subscription(JointState, args.joint_states_topic, self._cb_js, 10)
        self.create_subscription(JointState, args.gripper_topic, self._cb_grip, 10)
        if args.lambda_source == "fd":
            self.create_subscription(JointState, args.fd_joint_topic, self._cb_fd, 10)
        else:
            self.create_subscription(WrenchStamped, args.lambda_wrench_topic, self._cb_l7w, 10)

        self.create_timer(1.0 / args.rate, self._on_tick)
        self.get_logger().info(f"logging -> {args.out} at {args.rate:.0f} Hz "
                               f"(lambda source: {args.lambda_source})")

    def _cb_pose(self, m): self.pose = m
    def _cb_wr(self, m): self.wr = m
    def _cb_js(self, m): self.js = m
    def _cb_grip(self, m): self.grip = m
    def _cb_fd(self, m): self.fd = m
    def _cb_l7w(self, m): self.l7w = m

    def _ee(self, t):
        if self.pose is None:
            return [NAN] * 3, [NAN] * 3
        p = self.pose.pose.position
        pos = [p.x, p.y, p.z]
        vel = [NAN] * 3
        if self._prev_p is not None:
            t0, x0, y0, z0 = self._prev_p
            dt = t - t0
            if dt > 1e-6:
                vel = [(p.x - x0) / dt, (p.y - y0) / dt, (p.z - z0) / dt]
        self._prev_p = (t, p.x, p.y, p.z)
        return pos, vel

    def _lambda(self):
        """Return (F[3], T[3], gripper_angle, gripper_force)."""
        if self.args.lambda_source == "fd":
            if self.fd is None:
                return [NAN] * 3, [NAN] * 3, NAN, NAN
            eff = list(self.fd.effort)
            F = (eff + [NAN] * 3)[:3]
            T = (eff[3:6] + [NAN] * 3)[:3] if len(eff) >= 6 else [NAN] * 3
            emap = _joint_effort_map(self.fd)
            pmap = {nm: (self.fd.position[i] if i < len(self.fd.position) else NAN)
                    for i, nm in enumerate(self.fd.name)}
            gang = pmap.get("fd_clutch", NAN)
            gfor = emap.get("fd_clutch", eff[-1] if eff else NAN)
            return F, T, gang, gfor
        else:
            if self.l7w is None:
                return [NAN] * 3, [NAN] * 3, NAN, NAN
            f, t = self.l7w.wrench.force, self.l7w.wrench.torque
            return [f.x, f.y, f.z], [t.x, t.y, t.z], NAN, NAN

    def _on_tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        pos, vel = self._ee(t)

        q = _by_name(self.js, "fr3_joint", "position", 7) if self.js else [NAN] * 7
        dq = _by_name(self.js, "fr3_joint", "velocity", 7) if self.js else [NAN] * 7

        gw = (self.grip.position[0] + self.grip.position[1]) if (
            self.grip and len(self.grip.position) >= 2) else NAN

        if self.wr is not None:
            f, tq = self.wr.wrench.force, self.wr.wrench.torque
            wr = [f.x, f.y, f.z, tq.x, tq.y, tq.z]
        else:
            wr = [NAN] * 6

        lF, lT, lga, lgf = self._lambda()

        self.w.writerow([t] + pos + vel + q + dq + [gw] + wr + lF + lT + [lga, lgf])

    def close(self):
        try:
            self.f.flush(); self.f.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description="Log FR3 + lambda.7 to CSV (ROS 2 Jazzy stack).")
    ap.add_argument("--out", default="fr3_lambda7_log.csv")
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--franka-pose-topic", default="/franka_robot_state_broadcaster/current_pose")
    ap.add_argument("--franka-wrench-topic",
                    default="/franka_robot_state_broadcaster/external_wrench_in_base_frame")
    ap.add_argument("--joint-states-topic", default="/joint_states")
    ap.add_argument("--gripper-topic", default="/fr3_gripper/joint_states")
    ap.add_argument("--lambda-source", choices=["fd", "wrench"], default="fd",
                    help="fd = forcedimension_ros2 /fd/joint_states effort; wrench = DHD fallback topic")
    ap.add_argument("--fd-joint-topic", default="/fd/joint_states")
    ap.add_argument("--lambda-wrench-topic", default="/lambda7/wrench")
    args, _ = ap.parse_known_args()

    rclpy.init(args=sys.argv)
    node = Logger(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
