#!/usr/bin/env python3
"""Capture camera-only RGB-D evidence without importing or controlling DimOS.

Run with `uv run --no-project --with numpy --with pyrealsense2 python ...`.
Output must be a new directory and is suitable for `local-setup/evidence/`.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pyrealsense2 as rs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=18)
    args = parser.parse_args()
    if args.samples < 3 or args.output.exists():
        parser.error("--samples must be >= 3 and --output must not exist")
    args.output.mkdir(parents=True)
    pipeline, config = rs.pipeline(), rs.config()
    config.enable_device(args.serial)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
    pipeline.start(config)
    try:
        align = rs.align(rs.stream.color)
        for _ in range(30):
            pipeline.wait_for_frames(5000)
        colors, depths = [], []
        for _ in range(args.samples):
            frames = align.process(pipeline.wait_for_frames(5000))
            colors.append(np.asanyarray(frames.get_color_frame().get_data()).copy())
            depths.append(np.asanyarray(frames.get_depth_frame().get_data()).copy())
        np.savez_compressed(args.output / "capture.npz", color_bgr=np.stack(colors), depth_raw=np.stack(depths))
        (args.output / "capture.json").write_text(json.dumps({"serial": args.serial, "profile": [640, 480, 6]}, indent=2) + "\n")
    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()
