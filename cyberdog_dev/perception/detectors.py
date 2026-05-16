from __future__ import annotations

"""Unified detector registry.

All camera-based HSV detectors are configured here. Each entry maps a
detector name to its ROS topic, HSV config key, and detection parameters.

Usage (programmatic):
    from perception.detectors import run
    run("red_pole")

Usage (CLI):
    python -m perception red_pole
    python -m perception list
"""

import argparse
import sys

from perception.color_detector import run_object_detector, run_scalar_detector

# name -> config dict
#   type="obj":    object detector with bounding box + distance estimation
#   type="scalar": scalar detector (lane edges, dashed lines)
_REGISTRY: dict[str, dict] = {
    "orange_ball":    dict(type="obj",    topic="/perception/orange_ball",    hsv_key="orange_ball",   label="orange_ball", width=0.2),
    "football":       dict(type="obj",    topic="/perception/football",       hsv_key="football_dark", label="football",    width=0.2),
    "coke":           dict(type="obj",    topic="/perception/coke",           hsv_key="coke",          label="coke",        width=0.12),
    "red_pole":       dict(type="obj",    topic="/perception/red_pole",       hsv_key="red_pole",      label="red_pole",    width=0.1),
    "block_obstacle": dict(type="obj",    topic="/perception/block_obstacle", hsv_key="gray_block",    label="block",       width=0.2),
    "lane_edge":      dict(type="scalar", topic="/perception/lane_edge"),
    "dashed_line":    dict(type="scalar", topic="/perception/dashed_line"),
}


def run(detector_name: str, args=None):
    """Launch a ROS2 detector node by name."""
    if detector_name not in _REGISTRY:
        raise ValueError(f"Unknown detector: {detector_name}. Available: {list(_REGISTRY)}")
    entry = _REGISTRY[detector_name]
    node_name = f"perception_{detector_name}"
    if entry["type"] == "scalar":
        run_scalar_detector(node_name, entry["topic"], args=args)
    else:
        run_object_detector(
            node_name, entry["topic"],
            entry["hsv_key"], entry["label"], entry["width"], args=args,
        )


def list_detectors() -> list[str]:
    """Return all registered detector names."""
    return list(_REGISTRY)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m perception",
        description="Launch a perception detector node",
    )
    sub = parser.add_subparsers(dest="detector")
    sub.add_parser("list", help="List all available detectors")
    for name, entry in _REGISTRY.items():
        p = sub.add_parser(name, help=f"Launch {name} detector ({entry['type']})")
        p.add_argument("--mode", choices=["sim", "real"], default="real")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.detector is None or args.detector == "list":
        for name, entry in _REGISTRY.items():
            print(f"  {name:20s}  [{entry['type']}]  {entry['topic']}")
        return
    run_args = ["--mode", args.mode]
    run(args.detector, run_args)
