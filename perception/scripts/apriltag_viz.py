#!/usr/bin/env python3
"""
Real-time AprilTag visualizer.

Subscribes to the camera color image and the apriltag_ros `/detections` topic,
draws the tag bounding box / id on the live video, and looks up each tag's 3D
pose from TF to overlay its position (x, y, z in metres) and orientation
(roll, pitch, yaw in degrees).

Run (after the apriltag launch is already running):
    source /opt/ros/jazzy/setup.bash
    python3 scripts/apriltag_viz.py

Useful overrides:
    python3 scripts/apriltag_viz.py --ros-args \
        -p image_topic:=/camera/camera/color/image_raw \
        -p detections_topic:=/detections
"""

import math

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge

import tf2_ros


def quat_to_euler_deg(x, y, z, w):
    """Quaternion -> (roll, pitch, yaw) in degrees (XYZ / ZYX aerospace convention)."""
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)  # use 90 deg if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class AprilTagViz(Node):
    def __init__(self):
        super().__init__("apriltag_viz")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("window_name", "AprilTag")
        # how long (s) a detection overlay survives without a fresh detection msg
        self.declare_parameter("detection_timeout", 0.5)

        image_topic = self.get_parameter("image_topic").value
        det_topic = self.get_parameter("detections_topic").value
        self.window_name = self.get_parameter("window_name").value
        self.det_timeout = float(self.get_parameter("detection_timeout").value)

        self.bridge = CvBridge()
        self.latest_dets = None
        self.latest_dets_time = None
        self.image_topic = image_topic
        self.frames_seen = 0
        self.warned_no_frames = False

        # TF: apriltag_ros broadcasts each tag at "<family>:<id>" e.g. tag36h11:0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Image, image_topic, self.image_cb, qos)
        self.create_subscription(
            AprilTagDetectionArray, det_topic, self.det_cb, qos
        )

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.get_logger().info(
            f"AprilTag viz running. image='{image_topic}' detections='{det_topic}'. "
            f"Press 'q' in the window or Ctrl+C to quit."
        )

        # GUI/watchdog tick so a window always appears, even before any frame
        # arrives, and so we warn loudly if the camera isn't publishing.
        self.create_timer(0.5, self.tick)

    def tick(self):
        """Keep the window responsive; show a status screen until frames arrive."""
        if self.frames_seen == 0:
            placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(
                placeholder, "Waiting for camera frames...", (20, 160),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA,
            )
            cv2.putText(
                placeholder, self.image_topic, (20, 200),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA,
            )
            cv2.imshow(self.window_name, placeholder)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                rclpy.shutdown()
            if not self.warned_no_frames:
                self.warned_no_frames = True
                self.get_logger().warn(
                    f"No frames yet on '{self.image_topic}'. Is the camera "
                    f"actually streaming? Check: ros2 topic hz {self.image_topic}"
                )

    def det_cb(self, msg: AprilTagDetectionArray):
        self.latest_dets = msg
        self.latest_dets_time = self.get_clock().now()

    def lookup_pose(self, family: str, tag_id: int, parent_frame: str):
        """Return (tx,ty,tz, roll,pitch,yaw) in metres/deg, or None if unavailable."""
        child = f"{family}:{tag_id}"
        try:
            tf = self.tf_buffer.lookup_transform(
                parent_frame, child, rclpy.time.Time()
            )
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        r, p, y = quat_to_euler_deg(q.x, q.y, q.z, q.w)
        return t.x, t.y, t.z, r, p, y

    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        self.frames_seen += 1
        dets = self.latest_dets
        fresh = (
            dets is not None
            and self.latest_dets_time is not None
            and (self.get_clock().now() - self.latest_dets_time).nanoseconds * 1e-9
            < self.det_timeout
        )

        n_drawn = 0
        if fresh:
            parent_frame = dets.header.frame_id
            for det in dets.detections:
                n_drawn += 1
                pts = np.array(
                    [[c.x, c.y] for c in det.corners], dtype=np.int32
                ).reshape(-1, 1, 2)

                # bounding polygon + centre
                cv2.polylines(
                    frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2
                )
                cx, cy = int(det.centre.x), int(det.centre.y)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

                # label anchor = top-most corner
                xs = [int(c.x) for c in det.corners]
                ys = [int(c.y) for c in det.corners]
                ax, ay = min(xs), min(ys)

                lines = [f"{det.family} id:{det.id}"]
                pose = self.lookup_pose(det.family, det.id, parent_frame)
                if pose is not None:
                    tx, ty, tz, r, p, yw = pose
                    lines.append(f"pos[m] x:{tx:+.3f} y:{ty:+.3f} z:{tz:+.3f}")
                    lines.append(f"rpy[deg] r:{r:+.1f} p:{p:+.1f} y:{yw:+.1f}")
                else:
                    lines.append("pose: (no TF)")

                self._draw_label(frame, lines, ax, ay)

        # HUD
        status = f"tags: {n_drawn}" if fresh else "tags: 0 (no detections)"
        cv2.putText(
            frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
            (0, 255, 255), 2, cv2.LINE_AA,
        )

        # XYZ reference frame (camera optical convention) on the right side
        self._draw_axes_gizmo(frame)

        cv2.imshow(self.window_name, frame)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            rclpy.shutdown()

    @staticmethod
    def _draw_axes_gizmo(frame):
        """Draw a fixed XYZ axis legend (camera optical frame) on the right side.

        Camera optical frame (REP 103): +X right, +Y down, +Z forward (into the
        scene). These are the axes the printed position/orientation values use.
        """
        h, w = frame.shape[:2]
        # origin of the gizmo, anchored to the right-centre of the image
        ox, oy = w - 95, h // 2
        L = 55  # axis length in px

        # panel background for readability
        x0, y0, x1, y1 = w - 165, oy - 80, w - 8, oy + 90
        overlay = frame.copy()
        cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

        cv2.putText(
            frame, "cam frame", (x0 + 8, y0 + 20), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (255, 255, 255), 1, cv2.LINE_AA,
        )

        font = cv2.FONT_HERSHEY_SIMPLEX
        # X -> right (red, BGR)
        cv2.arrowedLine(frame, (ox, oy), (ox + L, oy), (0, 0, 255), 2,
                        tipLength=0.25)
        cv2.putText(frame, "X", (ox + L + 4, oy + 5), font, 0.6,
                    (0, 0, 255), 2, cv2.LINE_AA)
        # Y -> down (green)
        cv2.arrowedLine(frame, (ox, oy), (ox, oy + L), (0, 255, 0), 2,
                        tipLength=0.25)
        cv2.putText(frame, "Y", (ox - 4, oy + L + 18), font, 0.6,
                    (0, 255, 0), 2, cv2.LINE_AA)
        # Z -> forward/into scene, drawn as a diagonal (blue)
        zx, zy = ox - int(L * 0.6), oy - int(L * 0.6)
        cv2.arrowedLine(frame, (ox, oy), (zx, zy), (255, 100, 0), 2,
                        tipLength=0.25)
        cv2.putText(frame, "Z", (zx - 18, zy + 2), font, 0.6,
                    (255, 100, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, "(fwd)", (zx - 30, zy + 18), font, 0.4,
                    (255, 100, 0), 1, cv2.LINE_AA)
        # origin dot
        cv2.circle(frame, (ox, oy), 3, (255, 255, 255), -1)

    @staticmethod
    def _draw_label(frame, lines, x, y):
        """Draw a stack of text lines with a dark background above point (x, y)."""
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale, thick = 0.5, 1
        pad = 4
        sizes = [cv2.getTextSize(t, font, scale, thick)[0] for t in lines]
        lh = max(s[1] for s in sizes) + 6
        box_w = max(s[0] for s in sizes) + 2 * pad
        box_h = lh * len(lines) + pad

        top = max(0, y - box_h - 4)
        left = max(0, x)
        overlay = frame.copy()
        cv2.rectangle(
            overlay, (left, top), (left + box_w, top + box_h), (0, 0, 0), -1
        )
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

        ty = top + lh - 4
        for t in lines:
            cv2.putText(
                frame, t, (left + pad, ty), font, scale,
                (255, 255, 255), thick, cv2.LINE_AA,
            )
            ty += lh


def main():
    rclpy.init()
    node = AprilTagViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
