#!/usr/bin/env python3
"""
record_rosbag.py  --  ROS 2: record FR3 + lambda.7 topics into a rosbag2 bag.

Wraps `ros2 bag record` (robust across distros). By default records the curated
topic set below; --all records every topic; --topics overrides the list. Ctrl-C
cleanly finalises the bag.

Examples:
  python3 record_rosbag.py -o demo_001                         # curated topics
  python3 record_rosbag.py -o demo_001 --storage mcap          # mcap (Iron+/plugin)
  python3 record_rosbag.py -o demo_001 --all                   # everything
  python3 record_rosbag.py -o demo_001 --topics /lambda7/wrench /fr3_gripper/joint_states

Replay / inspect later:
  ros2 bag info demo_001
  ros2 bag play demo_001
"""
import argparse
import signal
import subprocess
import sys

DEFAULT_TOPICS = [
    # --- FR3 (franka_robot_state_broadcaster; these are BEST-EFFORT QoS) ---
    "/franka_robot_state_broadcaster/current_pose",
    "/franka_robot_state_broadcaster/external_wrench_in_base_frame",
    "/franka_robot_state_broadcaster/robot_state",   # full state (if you also want it)
    "/joint_states",                                 # arm joint angles/velocities
    "/fr3_gripper/joint_states",                     # Franka Hand width
    # --- lambda.7 (forcedimension_ros2) ---
    "/fd/ee_pose",
    "/fd/button_state",
    "/fd/joint_states",                              # position/velocity/EFFORT (rendered force)
    "/fd/fd_controller/commands",                    # commanded force
    # --- control target and scene (not in the CSV schema) ---
    "/cartesian_impedance_example_controller/equilibrium_pose",
    "/detections",                                   # AprilTag 2D detections
    "/tf",                                           # tag poses live here, as <family>:<id>
    "/tf_static",
]

# NOTE: `ros2 bag record` subscribes with the publisher's QoS, so the best-effort
# franka topics are captured fine. Verify names first with `ros2 topic list`.


def main():
    ap = argparse.ArgumentParser(description="Record FR3 + lambda.7 topics to a rosbag2 bag.")
    ap.add_argument("--out", "-o", default="fr3_lambda7_bag", help="output bag directory")
    ap.add_argument("--all", action="store_true", help="record ALL topics (-a)")
    ap.add_argument("--topics", nargs="*", default=None, help="explicit topic list (overrides default)")
    ap.add_argument("--storage", "-s", default="sqlite3", help="storage id: sqlite3 (default) | mcap")
    args, _ = ap.parse_known_args()

    cmd = ["ros2", "bag", "record", "-o", args.out, "-s", args.storage]
    if args.all:
        cmd.append("-a")
    else:
        cmd += (args.topics if args.topics else DEFAULT_TOPICS)

    print("running:", " ".join(cmd), flush=True)
    try:
        proc = subprocess.Popen(cmd)
    except FileNotFoundError:
        sys.exit("error: `ros2` not found -- source your ROS 2 environment first.")

    # forward Ctrl-C / SIGTERM to the recorder so the bag is finalised cleanly
    def _forward(sig, _frame):
        proc.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, _forward)
    signal.signal(signal.SIGTERM, _forward)
    proc.wait()


if __name__ == "__main__":
    main()
