#!/usr/bin/env python3
"""
check_log.py -- sanity-check a recorded CSV before you use it for anything.

Reports the sampling rate actually achieved, timing gaps, how much of each
signal group is missing, and per-tag visibility. Run it right after a session:
a recording where the FR3 wrench is 100% NaN, or where the camera dropped out
halfway, is much cheaper to notice now than three weeks later.

    python3 check_log.py logs/all_tasks/all_tasks_20260729_150004_001.csv
    python3 check_log.py logs/all_tasks/*.csv --gap 0.2

Needs pandas. No ROS environment required.
"""

import argparse
import sys

try:
    import numpy as np
    import pandas as pd
except ImportError:
    sys.exit("needs numpy and pandas:  pip install numpy pandas")


GROUPS = {
    "FR3 pose": ["ee_x", "ee_y", "ee_z", "ee_qx", "ee_qy", "ee_qz", "ee_qw"],
    "FR3 joints": [f"q{i}" for i in range(1, 8)] + [f"dq{i}" for i in range(1, 8)],
    "FR3 wrench": ["fr3_Fx", "fr3_Fy", "fr3_Fz"],
    "gripper": ["gripper_width"],
    "lambda pose": ["fd_x", "fd_y", "fd_z", "fd_qx", "fd_qy", "fd_qz", "fd_qw"],
    "lambda force": ["fd_Fx", "fd_Fy", "fd_Fz"],
}


def tag_ids(columns):
    ids = set()
    for col in columns:
        if col.startswith("tag") and col.endswith("_x"):
            body = col[3:-2]
            if body.isdigit():
                ids.add(int(body))
    return sorted(ids)


def check(path, gap_threshold):
    df = pd.read_csv(path)
    n = len(df)
    print(f"\n{path}")
    print(f"  rows            {n}")

    if n < 2 or "t" not in df.columns:
        print("  not enough rows to analyse")
        return

    t = df["t"].to_numpy()
    dt = np.diff(t)
    duration = t[-1] - t[0]

    print(f"  duration        {duration:.1f} s")
    print(f"  mean rate       {(n - 1) / duration:.1f} Hz")
    print(
        f"  sample dt       median {np.median(dt) * 1e3:.1f} ms, "
        f"p99 {np.percentile(dt, 99) * 1e3:.1f} ms, "
        f"max {dt.max() * 1e3:.1f} ms"
    )

    if (dt <= 0).any():
        print(f"  WARNING         {(dt <= 0).sum()} non-increasing timestamps")

    gaps = np.flatnonzero(dt > gap_threshold)
    if gaps.size:
        print(f"  WARNING         {gaps.size} gaps over {gap_threshold * 1e3:.0f} ms")
        for i in gaps[:5]:
            print(f"                    t+{t[i] - t[0]:.2f}s  {dt[i] * 1e3:.0f} ms")
        if gaps.size > 5:
            print(f"                    ... and {gaps.size - 5} more")

    print("  signal coverage (share of rows with a value)")
    for name, cols in GROUPS.items():
        present = [c for c in cols if c in df.columns]
        if not present:
            print(f"    {name:<14} columns absent")
            continue
        coverage = 1.0 - df[present].isna().to_numpy().mean()
        flag = "  <-- check this" if coverage < 0.5 else ""
        print(f"    {name:<14} {coverage * 100:5.1f} %{flag}")

    ids = tag_ids(df.columns)
    if ids:
        print("  AprilTag visibility")
        for tag in ids:
            col = f"tag{tag}_x"
            visible = 1.0 - df[col].isna().mean()
            print(f"    tag {tag:<10} {visible * 100:5.1f} %")
    else:
        print("  AprilTag        no tag columns (two-signal recording)")


def main():
    parser = argparse.ArgumentParser(description="Sanity-check FR3/lambda.7 CSV logs.")
    parser.add_argument("paths", nargs="+", help="CSV files to check")
    parser.add_argument(
        "--gap",
        type=float,
        default=0.1,
        help="report sample gaps longer than this many seconds (default 0.1)",
    )
    args = parser.parse_args()

    for path in args.paths:
        try:
            check(path, args.gap)
        except Exception as exc:
            print(f"\n{path}\n  failed: {exc}")
    print()


if __name__ == "__main__":
    main()
