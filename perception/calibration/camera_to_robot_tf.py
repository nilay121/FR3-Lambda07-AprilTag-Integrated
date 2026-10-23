#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from collections import deque
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster


def transform_to_matrix(transform):
    """Convert geometry_msgs/Transform to a 4x4 matrix."""
    translation = transform.translation
    rotation = transform.rotation

    matrix = np.eye(4)

    matrix[:3, :3] = Rotation.from_quat([
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    ]).as_matrix()

    matrix[:3, 3] = [
        translation.x,
        translation.y,
        translation.z,
    ]

    return matrix


class CameraToRobotTF(Node):
    def __init__(self):
        super().__init__("camera_to_robot_tf")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Known pose of reference tag 0 in the robot base frame.
        tag_position = np.array([
            -0.132484000,
            -0.337336000,
            0.277611000,
        ])

        tag_quaternion = np.array([
            0.506264241,
            0.492169024,
            0.512785820,
            0.488381892,
        ])

        self.T_base_tag = np.eye(4)
        self.T_base_tag[:3, :3] = Rotation.from_quat(
            tag_quaternion
        ).as_matrix()
        self.T_base_tag[:3, 3] = tag_position

        self.last_camera_transform = None
        self.first_success = True

        # Number of observations to average
        self.num_samples = 50

        # Store the last N camera transforms
        self.camera_positions = deque(maxlen=self.num_samples)
        self.camera_rotations = deque(maxlen=self.num_samples)

        self.create_timer(0.1, self.update_transform)

    def update_transform(self):

        if (
            self.last_camera_transform is not None
            and not self.first_success
        ):
            self.publish_camera_transform()
            return

        try:
            # Pose of reference tag 0 in the camera optical frame.
            camera_to_tag = self.tf_buffer.lookup_transform(
                "camera_color_optical_frame",
                "tag36h11:0",
                Time(),
            )

            # RealSense internal transform.
            camera_link_to_optical = self.tf_buffer.lookup_transform(
                "camera_link",
                "camera_color_optical_frame",
                Time(),
            )

            T_camera_optical_tag = transform_to_matrix(
                camera_to_tag.transform
            )

            T_camera_link_optical = transform_to_matrix(
                camera_link_to_optical.transform
            )

            # base -> optical camera frame
            T_base_camera_optical = (
                self.T_base_tag
                @ np.linalg.inv(T_camera_optical_tag)
            )

            # base -> camera_link
            T_base_camera_link = (
                T_base_camera_optical
                @ np.linalg.inv(T_camera_link_optical)
            )

            # Store current estimate
            self.camera_positions.append(
                T_base_camera_link[:3, 3]
            )

            self.camera_rotations.append(
                Rotation.from_matrix(
                    T_base_camera_link[:3, :3]
                )
            )

            # Wait until enough samples have been collected
            if len(self.camera_positions) < self.num_samples:
                remaining = self.num_samples - len(self.camera_positions)

                if remaining % 10 == 0:
                    self.get_logger().info(
                        f"Collecting calibration samples "
                        f"({len(self.camera_positions)}/{self.num_samples})"
                    )

                return

            # ---------- Average translation ----------
            avg_position = np.mean(
                np.asarray(self.camera_positions),
                axis=0,
            )

            # ---------- Average rotation ----------
            rot_mats = np.stack(
                [r.as_matrix() for r in self.camera_rotations],
                axis=0,
            )

            avg_rot = np.mean(rot_mats, axis=0)

            U, _, Vt = np.linalg.svd(avg_rot)
            avg_rot = U @ Vt

            avg_transform = np.eye(4)
            avg_transform[:3, :3] = avg_rot
            avg_transform[:3, 3] = avg_position

            self.last_camera_transform = avg_transform

            if self.first_success:
                self.get_logger().info(
                    "Camera calibration completed."
                )
                self.first_success = False

        except Exception as error:
            if self.last_camera_transform is None:
                self.get_logger().warning(
                    f"Waiting for reference tag 0 and camera TF: {error}",
                    throttle_duration_sec=2.0,
                )
                return

        self.publish_camera_transform()

    def publish_camera_transform(self):
        matrix = self.last_camera_transform

        if matrix is None:
            return

        quaternion = Rotation.from_matrix(
            matrix[:3, :3]
        ).as_quat()

        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base"
        message.child_frame_id = "camera_link"

        message.transform.translation.x = float(matrix[0, 3])
        message.transform.translation.y = float(matrix[1, 3])
        message.transform.translation.z = float(matrix[2, 3])

        message.transform.rotation.x = float(quaternion[0])
        message.transform.rotation.y = float(quaternion[1])
        message.transform.rotation.z = float(quaternion[2])
        message.transform.rotation.w = float(quaternion[3])

        self.tf_broadcaster.sendTransform(message)


def main():
    rclpy.init()

    node = CameraToRobotTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()