# logging/

Synchronised CSV logging for the FR3 and the lambda.7, with AprilTag poses as an
optional third stream. One row per sample, one clock, everything side by side.

| File | What it is |
|---|---|
| `sync_logger.py` | Standalone logger. Records FR3 + lambda.7, and tags with `--tags`. Same column schema as the bridge. Use it for any session the bridge is not driving. |
| `check_log.py` | Reads a finished CSV and reports achieved rate, timing gaps, per-signal coverage and per-tag visibility. Needs pandas, not ROS. Run it after every session. |
| `fr3_lambda7_logger.py` | The original FR3 + lambda.7 logger. Different, narrower schema — it has EE linear velocity and the full 6-DOF wrench but no tags. Kept because earlier recordings use it. |
| `record_rosbag.py` | Wrapper around `ros2 bag record` for a curated topic set, if you want raw topics instead of aligned rows. |

Quick reference:

```bash
# FR3 + lambda.7, no camera
python3 sync_logger.py --task-type handguided --rate 100 --no-tags

# FR3 + lambda.7 + tags, poses in the robot base frame
python3 sync_logger.py --task-type pick_glass --rate 100 --tags --tag-ids 10 11 12

# check what you recorded
python3 check_log.py logs/pick_glass/*.csv
```

Files land in `<--log-dir>/logs/<--task-type>/<task-type>_<timestamp>_<NNN>.csv`.

Full documentation, including the column schema and what "synchronised" does and
does not guarantee: [../docs/04_logging.md](../docs/04_logging.md).
