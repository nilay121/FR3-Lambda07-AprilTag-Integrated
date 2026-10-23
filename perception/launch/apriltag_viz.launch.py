import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    this_dir = os.path.dirname(os.path.realpath(__file__))
    ws_root = os.path.dirname(this_dir)
    viz_script = os.path.join(ws_root, "scripts", "apriltag_viz.py")

    return LaunchDescription([
        # Bring up the existing apriltag detector (camera must already be running).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(this_dir, "apriltag_realsense.launch.py")
            )
        ),
        # Live OpenCV visualizer with bounding box + pose overlay.
        ExecuteProcess(
            cmd=["python3", viz_script],
            output="screen",
        ),
    ])
