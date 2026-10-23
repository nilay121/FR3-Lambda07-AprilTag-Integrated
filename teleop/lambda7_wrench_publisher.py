#!/usr/bin/env python3
"""
lambda7_wrench_publisher.py  --  ROS 2 (rclpy) node  [OPTIONAL FALLBACK]

Only needed if you are NOT using the forcedimension_ros2 driver, or if that driver
does not expose the Lambda.7 force in /fd/joint_states effort. It talks to the raw
DHD SDK and publishes /lambda7/wrench. To log from it instead of /fd, run the logger
with:  --lambda-source wrench --lambda-wrench-topic /lambda7/wrench

If you already run forcedimension_ros2 (the recommended path), ignore this file --
the logger reads Lambda.7 force from /fd/joint_states by default.

Publishes the Force Dimension lambda.7 device state:
    /lambda7/wrench          geometry_msgs/msg/WrenchStamped  (force + torque on handle)
    /lambda7/pose            geometry_msgs/msg/PoseStamped    (device end-point position)
    /lambda7/gripper_force   std_msgs/msg/Float64             (gripper opening force)

IMPORTANT — what "force" means: Force Dimension devices have no load cell; they RENDER
forces with their motors and report the force applied to the handle. In bilateral
teleop that equals the reflected FR3 wrench. Log it together with the FR3-side
o_f_ext_hat_k (the true object-interaction estimate).

Requires the Force Dimension SDK + its Python binding. The binding's module name /
signatures vary; the SDK calls are isolated in _Device and marked (1)(2)(3) -- adjust
to your install.

Run:  python3 lambda7_wrench_publisher.py --rate 1000
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PoseStamped
from std_msgs.msg import Float64

# ---- SDK import (ADJUST to your Force Dimension Python binding) --------------------
try:
    from forcedimension_core import dhd            # <-- (1) common: forcedimension_core
except Exception:
    try:
        import dhd                                 # <-- (2) classic swig binding
    except Exception as e:                         # pragma: no cover
        dhd = None
        _IMPORT_ERR = e
# -----------------------------------------------------------------------------------


class _Device:
    def open(self, logger):
        if dhd is None:
            raise RuntimeError("Force Dimension Python binding not found: %s" % _IMPORT_ERR)
        if dhd.open() < 0:                          # <-- (3) open first available device
            raise RuntimeError("dhd.open() failed -- device connected/powered? udev rules set?")
        name = dhd.getSystemName() if hasattr(dhd, "getSystemName") else "Force Dimension device"
        logger.info("opened %s" % name)

    def read(self):
        ft = dhd.getForceAndTorqueAndGripperForce()      # (fx,fy,fz,tx,ty,tz,gf)
        vals = [v for v in ft if isinstance(v, float)]
        fx, fy, fz, tx, ty, tz, gf = (vals + [0.0] * 7)[:7]
        pos = dhd.getPosition()                          # (px,py,pz)
        pvals = [v for v in pos if isinstance(v, float)]
        px, py, pz = (pvals + [0.0] * 3)[:3]
        return (fx, fy, fz), (tx, ty, tz), gf, (px, py, pz)

    def close(self):
        try:
            dhd.close()
        except Exception:
            pass


class Lambda7Publisher(Node):
    def __init__(self, args):
        super().__init__("lambda7_wrench_publisher")
        self.frame = args.frame
        self.pub_w = self.create_publisher(WrenchStamped, "/lambda7/wrench", 10)
        self.pub_p = self.create_publisher(PoseStamped, "/lambda7/pose", 10)
        self.pub_g = self.create_publisher(Float64, "/lambda7/gripper_force", 10)
        self.dev = _Device()
        self.dev.open(self.get_logger())
        self.create_timer(1.0 / args.rate, self._on_tick)

    def _on_tick(self):
        try:
            (fx, fy, fz), (tx, ty, tz), gf, (px, py, pz) = self.dev.read()
        except Exception as e:
            self.get_logger().warn("device read failed: %s" % e, throttle_duration_sec=2.0)
            return
        now = self.get_clock().now().to_msg()

        w = WrenchStamped()
        w.header.stamp = now
        w.header.frame_id = self.frame
        w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = fx, fy, fz
        w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z = tx, ty, tz
        self.pub_w.publish(w)

        p = PoseStamped()
        p.header.stamp = now
        p.header.frame_id = self.frame
        p.pose.position.x, p.pose.position.y, p.pose.position.z = px, py, pz
        p.pose.orientation.w = 1.0
        self.pub_p.publish(p)

        self.pub_g.publish(Float64(data=float(gf)))

    def close(self):
        self.dev.close()


def main():
    ap = argparse.ArgumentParser(description="Publish lambda.7 forces/pose (ROS 2).")
    ap.add_argument("--rate", type=float, default=1000.0)
    ap.add_argument("--frame", default="lambda7_base")
    args, _ = ap.parse_known_args()

    rclpy.init(args=sys.argv)
    node = Lambda7Publisher(args)
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
