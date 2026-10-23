#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

# Replace these with your measured values in metres
center = np.array([-0.132484, -0.337336, 0.277611], dtype=float)
right = np.array([-0.133051, -0.282646, 0.279717], dtype=float)
top = np.array([-0.132617, -0.339480, 0.330462], dtype=float)



class ReferenceTagTransformPublisher(Node):
    def __init__(self, position, quaternion):
        super().__init__("reference_tag_transform_publisher")

        self.broadcaster = StaticTransformBroadcaster(self)

        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base"
        transform.child_frame_id = "reference_tag_0_known"

        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])

        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])

        self.broadcaster.sendTransform(transform)

        self.get_logger().info(
            "Published static transform: base -> reference_tag_0_known"
        )


def normalize(vector):
    norm = np.linalg.norm(vector)

    if norm < 1e-9:
        raise ValueError("Two measured points are too close together.")

    return vector / norm


def main():
    
    # Tag X axis: center toward right edge.
    x_axis = normalize(right - center)

    # Temporary Y axis: center toward top edge.
    y_approx = normalize(top - center)

    # Make a right-handed orthogonal coordinate system.
    z_axis = normalize(np.cross(x_axis, y_approx))
    y_axis = normalize(np.cross(z_axis, x_axis))

    # Columns are the tag axes expressed in the robot base frame.
    rotation_matrix = np.column_stack(
        (x_axis, y_axis, z_axis)
    )

    quaternion = Rotation.from_matrix(
        rotation_matrix
    ).as_quat()  # Returns x, y, z, w.


    print("Reference-tag position in base:")
    print(f"x: {center[0]:.9f}")
    print(f"y: {center[1]:.9f}")
    print(f"z: {center[2]:.9f}")

    print("\nReference-tag quaternion in base:")
    print(f"qx: {quaternion[0]:.9f}")
    print(f"qy: {quaternion[1]:.9f}")
    print(f"qz: {quaternion[2]:.9f}")
    print(f"qw: {quaternion[3]:.9f}")

    print("\nRotation matrix:")
    print(rotation_matrix)

    rclpy.init()

    node = ReferenceTagTransformPublisher(
        position=center,
        quaternion=quaternion,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()