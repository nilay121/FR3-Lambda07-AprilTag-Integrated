# data/

`sample/teleop_sample_log.csv` is the first 400 rows of a real teleoperation
recording (`all_tasks`, tags 10–14, ~29 Hz), committed so the column layout is
visible without running anything:

```bash
python3 ../logging/check_log.py sample/teleop_sample_log.csv
head -1 sample/teleop_sample_log.csv | tr ',' '\n' | nl
```

Note that `gripper_width` is NaN throughout — that is the known topic-name issue
described in the root README, not an artefact of the truncation.

Full recordings do not belong in git. `.gitignore` excludes `logs/` and `*.csv`
outside this folder; keep sessions on a share or a data server and reference them
by path. A one-minute teleoperation run at 30 Hz with five tags is roughly 3 MB.

Column meanings: [../docs/04_logging.md](../docs/04_logging.md) §4.2.
