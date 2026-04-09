#!/usr/bin/env python3
import os
import random
import tempfile
from typing import Iterable, List, Tuple

# Bright, distinct colors (R; G; B) for RViz Path display.
# Stored as "R; G; B" strings to match RViz's expected config format.
_PALETTE: List[Tuple[str, str]] = [
    ("255; 0; 0",       "Red"),
    ("0; 255; 0",       "Green"),
    ("0; 0; 255",       "Blue"),
    ("255; 255; 0",     "Yellow"),
    ("191; 255; 0",     "Lime"),
    ("255; 0; 255",     "Magenta"),
    ("0; 255; 255",     "Cyan"),
    ("255; 165; 0",     "Orange"),
    ("160; 32; 240",    "Purple"),
    ("0; 128; 128",     "Teal"),
    ("255; 105; 180",   "Pink"),
    ("255; 215; 0",     "Gold"),
    ("0; 127; 255",     "Azure"),
    ("255; 127; 80",    "Coral"),
    ("75; 0; 130",      "Indigo"),
]

def _pick_colors(n: int) -> List[Tuple[str, str]]:
    """Return `n` visually distinct colors from the palette (cycled if n > len)."""
    colors = _PALETTE[:]         # copy to avoid modifying the global palette
    random.shuffle(colors)       # randomize assignment across robots
    return [tuple(colors[i % len(colors)]) for i in range(n)]

def _goal_tool_lines(prefix, ids):
    return [
        line
        for rid in ids
        for line in [
            "    - Class: rviz_default_plugins/SetGoal",
           f"      Topic: /{prefix}{rid}/goal_pose",
        ]
    ]

def make_rviz_config(robot_ids: Iterable[str], prefix: str = "robot", map_topic: str = "/map") -> str:
    """
    Build a minimal RViz configuration for multi-robot visualization and
    return the absolute path to a generated temporary .rviz file.

    - Creates a *global* Map display configured to reliably latch and show the static map.
    - Adds a root "Robots" group, then a child Group per robot with:
        * TF (disabled by default to reduce clutter),
        * LaserScan (topic: /<prefix><id>/scan),
        * RobotModel (description: /<prefix><id>/robot_description),
        * Path (topic: /<prefix><id>/plan) using Billboards, width 0.05, colored distinctly.
    - The Fixed Frame is "map" and frame rate is 30 FPS.

    Args:
        robot_ids: Iterable of robot identifiers (e.g., ["123", "456"]).
        prefix: Namespace/name prefix for robots (e.g., "robot" --> "/robot123").
        map_topic: Topic for the global Map display (default "/map").

    Returns:
        Path to a temporary .rviz file containing the generated configuration.
    """
    # Normalize/clean IDs and assign distinct colors per robot
    ids = [str(r).strip() for r in robot_ids if str(r).strip()]
    colors = _pick_colors(len(ids))

    # Base RViz panels and global settings
    lines = [
        "Panels:",
        "  - Class: rviz_common/Displays",
        "    Name: Displays",
        "    Property Tree Widget:",
        "      Expanded:",
        "        - /Robots1",
        "      Splitter Ratio: 0.5",
        "  - Class: rviz_common/Selection",
        "    Name: Selection",
        "  - Class: rviz_common/Views",
        "    Name: Views",
        "  - Class: rviz_common/Tool Properties",
        "    Name: Tool Properties",
        "Visualization Manager:",
        "  Global Options:",
        "    Fixed Frame: map",
        "    Frame Rate: 30",
        "  Displays:",
        # Grid for orientation
        "    - Class: rviz_default_plugins/Grid",
        "      Name: Grid",
        "      Enabled: true",
        "      Plane Cell Count: 100",
        "      Reference Frame: map",
        # Static map with robust QoS that matches Nav2 map_server (TRANSIENT_LOCAL)
        "    - Class: rviz_default_plugins/Map",
        "      Name: Map",
        "      Enabled: true",
        "      Topic:",
        "        Depth: 5",
        "        Durability Policy: Transient Local",
        "        Filter size: 10",
        "        History Policy: Keep Last",
        "        Reliability Policy: Reliable",
       f"        Value: {map_topic}",
        # Root group that will contain one group per robot
        "    - Class: rviz_common/Group",
        "      Name: Robots",
        "      Enabled: true",
        "      Displays:",
    ]

    # Per-robot sub-groups with standard displays
    for idx, rid in enumerate(ids):
        ns = f"/{prefix}{rid}"   # e.g., "/robot123"
        (color, color_name) = colors[idx]
        lines += [
            "        - Class: rviz_common/Group",
           f"          Name: {prefix}{rid} ({color_name})",
            "          Enabled: true",
            "          Displays:",
            # TF is useful but disabled by default to reduce visual clutter
            "            - Class: rviz_default_plugins/TF",
            "              Name: TF",
            "              Enabled: false",
            # LaserScan
            "            - Class: rviz_default_plugins/LaserScan",
            "              Name: Scan",
            "              Enabled: true",
           f"              Topic: {ns}/scan",
            "              Size (m): 0.03",
            # RobotModel (URDF is expected to be published on /<ns>/robot_description)
            "            - Class: rviz_default_plugins/RobotModel",
            "              Name: RobotModel",
            "              Enabled: true",
           f"              Description Topic: {ns}/robot_description",
            # Global plan (or any nav path) as a billboard line with a fixed width
            "            - Class: rviz_default_plugins/Path",
            "              Name: Plan",
            "              Enabled: true",
           f"              Topic: {ns}/plan",
            "              Line Style: Billboards",
            "              Line Width: 0.05",
           f"              Color: {color}",
        ]

    # Toolbar tools and a simple default camera view
    lines += [
        "  Tools:",
        "    - Class: rviz_default_plugins/Interact",
        "    - Class: rviz_default_plugins/MoveCamera",
        "    - Class: rviz_default_plugins/Select",
        "    - Class: rviz_default_plugins/FocusCamera",
        "    - Class: rviz_default_plugins/Measure",
             *_goal_tool_lines(prefix, ids), # Adds a 2D Goal pose tool for each robot
        "    - Class: rviz_default_plugins/PublishPoint",
        "  Views:",
        "    Current:",
        "      Class: rviz_default_plugins/Orbit",
        "      Distance: 10",
        "      Pitch: 0.5",
        "      Yaw: 0.0",
    ]

    # Write to a temp .rviz file and return the path
    fd, path = tempfile.mkstemp(prefix="multi_robot_", suffix=".rviz")
    with os.fdopen(fd, "w") as f:
        f.write("\n".join(lines))
    return path
