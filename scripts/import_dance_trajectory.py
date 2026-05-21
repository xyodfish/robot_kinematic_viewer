#!/usr/bin/env python3
"""Convert /data/dance CSV trajectories to robot_kinematic_viewer playback format."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

# dance3: chassis_z is planar yaw (rad), not height
JOINT_ORDER = [
    "leg_joint1",
    "leg_joint2",
    "leg_joint3",
    "leg_joint4",
    "leg_joint5",
    "head_joint1",
    "head_joint2",
    "left_arm_joint1",
    "left_arm_joint2",
    "left_arm_joint3",
    "left_arm_joint4",
    "left_arm_joint5",
    "left_arm_joint6",
    "left_arm_joint7",
    "right_arm_joint1",
    "right_arm_joint2",
    "right_arm_joint3",
    "right_arm_joint4",
    "right_arm_joint5",
    "right_arm_joint6",
    "right_arm_joint7",
]

ALIASES = {
    "leg1": "leg_joint1",
    "leg2": "leg_joint2",
    "leg3": "leg_joint3",
    "leg4": "leg_joint4",
    "leg5": "leg_joint5",
    "head1": "head_joint1",
    "head2": "head_joint2",
    **{f"left_arm{i}": f"left_arm_joint{i}" for i in range(1, 8)},
    **{f"right_arm{i}": f"right_arm_joint{i}" for i in range(1, 8)},
}

TIME_KEYS = {"time", "t", "timestamp"}
SKIP_KEYS = {"idx", "id"}
BASE_X_KEYS = {"chassis_x", "base_x", "mobile_x"}
BASE_Y_KEYS = {"chassis_y", "base_y", "mobile_y"}
BASE_YAW_KEYS = {"chassis_yaw", "chassis_z", "base_yaw", "mobile_yaw"}


def norm_key(name: str) -> str:
    return name.strip().lower()


def norm_joint(name: str) -> str:
    k = norm_key(name)
    return ALIASES.get(k, name.strip())


def read_dance_rows(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"empty csv: {path}")
        field_map = {norm_key(n): n for n in reader.fieldnames}

        time_field = next((field_map[k] for k in TIME_KEYS if k in field_map), None)
        if time_field is None:
            raise ValueError(f"missing time/timestamp column in {path}")

        bx = next((field_map[k] for k in BASE_X_KEYS if k in field_map), None)
        by = next((field_map[k] for k in BASE_Y_KEYS if k in field_map), None)
        byaw = next((field_map[k] for k in BASE_YAW_KEYS if k in field_map), None)

        joint_fields: dict[str, str] = {}
        for raw in reader.fieldnames:
            nk = norm_key(raw)
            if nk in TIME_KEYS or nk in SKIP_KEYS or nk in BASE_X_KEYS or nk in BASE_Y_KEYS or nk in BASE_YAW_KEYS:
                continue
            joint_fields[norm_joint(raw)] = raw

        rows: list[dict[str, float]] = []
        for item in reader:
            row: dict[str, float] = {"time": float(item[time_field])}
            if bx:
                row["chassis_x"] = float(item[bx])
            if by:
                row["chassis_y"] = float(item[by])
            if byaw:
                row["chassis_yaw"] = float(item[byaw])
            for jn, field in joint_fields.items():
                row[jn] = float(item[field])
            rows.append(row)
    return rows


def downsample(rows: list[dict[str, float]], target_hz: float) -> list[dict[str, float]]:
    if len(rows) < 2 or target_hz <= 0:
        return rows
    duration = rows[-1]["time"] - rows[0]["time"]
    if duration <= 0:
        return rows[:1]
    source_hz = (len(rows) - 1) / duration
    step = max(1, int(round(source_hz / target_hz)))
    out = [rows[i] for i in range(0, len(rows), step)]
    if out[-1] is not rows[-1]:
        out.append(rows[-1])
    return out


def write_viewer_csv(rows: list[dict[str, float]], out_path: Path) -> None:
    has_base = any("chassis_x" in r and "chassis_y" in r and "chassis_yaw" in r for r in rows)
    header = ["time"]
    if has_base:
        header += ["chassis_x", "chassis_y", "chassis_yaw"]
    header += JOINT_ORDER

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        last: dict[str, float] = {}
        for row in rows:
            line = [f"{row['time']:.6g}"]
            if has_base:
                for k in ("chassis_x", "chassis_y", "chassis_yaw"):
                    if k in row:
                        last[k] = row[k]
                    line.append(f"{last.get(k, 0.0):.6g}")
            for jn in JOINT_ORDER:
                if jn in row:
                    last[jn] = row[jn]
                line.append(f"{last.get(jn, 0.0):.6g}")
            w.writerow(line)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", type=Path, help="dance repo csv (e.g. /data/dance/data/dance3/generate/pos/...)")
    parser.add_argument("output_csv", type=Path, help="viewer playback csv path")
    parser.add_argument("--hz", type=float, default=30.0, help="output sample rate (default 30)")
    args = parser.parse_args()

    rows = read_dance_rows(args.input_csv)
    rows = downsample(rows, args.hz)
    write_viewer_csv(rows, args.output_csv)
    print(f"imported {len(rows)} frames -> {args.output_csv} ({rows[-1]['time'] - rows[0]['time']:.2f}s)")


if __name__ == "__main__":
    main()
