#!/usr/bin/env python3
"""Sync simulator LiDAR calibration files from sharpa_platform."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def read_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        return list(reader.fieldnames or []), list(reader)


def write_rows(path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    repository = Path(__file__).resolve().parents[1]
    default_platform = repository.parent / "sharpa_platform"
    parser = argparse.ArgumentParser()
    parser.add_argument("--platform-root", type=Path, default=default_platform)
    args = parser.parse_args()

    platform_calibration = (
        args.platform_root / "plugins/sharpa_lidar_plugin/correction"
    )
    simulator_calibration = repository / "resources/lidar/calibration"

    jt_fields, jt_rows = read_rows(platform_calibration / "JT16_sample_angle.csv")
    if jt_fields != ["LaserID", "Elevation", "Azimuth"] or len(jt_rows) != 16:
        raise ValueError("platform JT16 calibration must contain 16 channel rows")
    write_rows(
        simulator_calibration / "JT16_sample_angle.csv", jt_fields, jt_rows
    )

    ftx_fields, ftx_rows = read_rows(
        platform_calibration / "FTX140_angle_calib_file.csv"
    )
    if ftx_fields != ["row", "col", "x", "y", "z"]:
        raise ValueError("unexpected platform FTX140 calibration columns")
    source = {(int(row["row"]), int(row["col"])): row for row in ftx_rows}
    if len(source) != 384 * 512:
        raise ValueError("platform FTX140 calibration must contain 384x512 directions")

    sampled = []
    for row in range(192):
        for col in range(256):
            source_row = source[(row * 2, col * 2)]
            sampled.append(
                {
                    "row": str(row),
                    "col": str(col),
                    "x": source_row["x"],
                    "y": source_row["y"],
                    "z": source_row["z"],
                }
            )
    write_rows(
        simulator_calibration / "FTX140_angle_calib_file.csv",
        ftx_fields,
        sampled,
    )


if __name__ == "__main__":
    main()
