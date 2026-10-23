# teleop/

| File | What it is |
|---|---|
| `lambda_fr3_bridge.py` | The bridge. lambda.7 pose → FR3 equilibrium pose, lambda.7 rotation → FR3 orientation, gripper axis → Franka Hand, FR3 wrench → handle force. Writes one wide-format CSV per teleoperation session. This is the one used for data collection. |
| `lambda_fr3_bridge_split_csv.py` | Earlier variant with the same control code, writing state and tags to two files joinable on `t` instead of one wide file. |
| `lambda7_wrench_publisher.py` | Fallback only. Talks to the raw Force Dimension SDK and publishes `/lambda7/wrench`, `/lambda7/pose`, `/lambda7/gripper_force`. Needed only if `forcedimension_ros2` is not in use or does not expose the device force. Ignore it on the normal path. |

Requires `numpy` and `scipy`. `apriltag_msgs` and `tf2_ros` are optional — without
them the bridge still runs and logs everything except tag poses.

Full documentation: [../docs/03_teleoperation.md](../docs/03_teleoperation.md).

Two things must be true before this works at all: the Franka controller patch in
`../patches/` is applied and built, and `Rmap` in `Bridge.__init__` matches your
physical lambda-to-FR3 axis alignment. Both are covered in the docs.
