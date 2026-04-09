#!/usr/bin/env python3
import argparse
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import matplotlib
matplotlib.use("TkAgg")  # or Qt5Agg if you prefer
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge

DEG = math.pi / 180.0

def clamp_angle_deg(a):
    # Normalizes an angle to [-180, 180)
    a = ((a + 180.0) % 360.0) - 180.0
    return a

class ScanBuffer:
    def __init__(self):
        # Holds the latest LaserScan with a lock for thread-safe access
        self.lock = threading.Lock()
        self.msg = None
        self.stamp_ns = 0

    def update(self, msg):
        # Stores a new message and its timestamp (ns) atomically
        with self.lock:
            self.msg = msg
            self.stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

    def get(self):
        # Returns the latest message and timestamp safely
        with self.lock:
            return self.msg, self.stamp_ns

class ScanNode(Node):
    def __init__(self, topic, buf):
        super().__init__('lidar_viz_sectors')
        self.buf = buf

        # Subscribes to the LaserScan topic with a LiDAR-friendly QoS (best-effort, small history)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(LaserScan, topic, self.cb, qos)
        # self.sub = self.create_subscription(LaserScan, topic, self.cb, 10)

    def cb(self, msg):
        # Updates the shared buffer whenever a new scan arrives
        self.buf.update(msg)

def scan_to_xy(msg, max_range=None):
    # Converts polar scan (r, θ) to Cartesian (x, y), with optional range clamping
    n = len(msg.ranges)
    angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
    r = np.array(msg.ranges, dtype=np.float32)
    r[~np.isfinite(r)] = np.inf
    if max_range is not None:
        r = np.minimum(r, max_range)
    x = r * np.cos(angles)
    y = r * np.sin(angles)
    return angles, r, x, y

def sector_masks(angles_deg, front_deg, side_deg, margin_deg):
    """
    Defines boolean masks for four sectors using degrees in [-180, 180):
      Front:       [-front_deg, +front_deg]
      Left:        [front_deg+margin,  side_deg]
      Right:       [-side_deg, -(front_deg+margin)]
      In-between:  Remaining angles (rear + gaps)
    """
    a = angles_deg
    f = front_deg
    s = side_deg
    m = margin_deg
    front = (a >= -f) & (a <= +f)

    left  = (a >= (f + m)) & (a <= s)
    right = (a <= -(f + m)) & (a >= -s)

    in_between = ~(front | left | right)
    return front, left, right, in_between

def classify_sector(angles_rad, ranges,
                    front_deg=30.0, side_deg=120.0, margin_deg=15.0,
                    range_min=None, range_max=None):
    # Classifies which sector is closest and returns per-sector minima and indices
    valid = np.isfinite(ranges)
    if range_min is not None:
        valid &= (ranges >= range_min)
    if range_max is not None:
        valid &= (ranges <= range_max)

    if not np.any(valid):
        return "no-returns", {"front": math.inf, "left": math.inf, "right": math.inf, "between": math.inf}, {}

    angles_deg = np.degrees(angles_rad)
    angles_deg = ((angles_deg + 180.0) % 360.0) - 180.0  # Normalize to [-180, 180)

    fmask, lmask, rmask, bmask = sector_masks(angles_deg, front_deg, side_deg, margin_deg)
    names = ["front", "left", "right", "between"]
    masks = [fmask, lmask, rmask, bmask]

    mins = {}
    idxs = {}
    for name, m in zip(names, masks):
        m2 = m & valid
        if not np.any(m2):
            mins[name] = math.inf
            idxs[name] = None
            continue
        # Finds the index of the minimum range within the sector
        m_indices = np.where(m2)[0]
        local_best_i = np.argmin(ranges[m2])
        best_idx = m_indices[local_best_i]
        mins[name] = float(ranges[best_idx])
        idxs[name] = int(best_idx)

    # Picks the sector with the smallest minimum distance
    label = min(mins, key=lambda k: mins[k])
    return label, mins, idxs

def main():
    # Parses CLI options for topic, sector angles, display settings, and headless mode
    ap = argparse.ArgumentParser(description="LiDAR live plot + sector classifier (no RViz).")
    ap.add_argument("--topic", default="/scan", help="LaserScan topic (default: /scan)")
    ap.add_argument("--front_deg", type=float, default=30.0, help="Half-width of front sector (deg)")
    ap.add_argument("--side_deg", type=float, default=120.0, help="Max angle for left/right sector (deg)")
    ap.add_argument("--margin_deg", type=float, default=15.0, help="Gap between front and side (deg)")
    ap.add_argument("--max_range", type=float, default=None, help="Clamp display range (m)")
    ap.add_argument("--rate", type=float, default=15.0, help="UI refresh rate (Hz)")
    ap.add_argument("--no_gui", action="store_true", help="Run headless: print classification to console only")
    args = ap.parse_args()

    rclpy.init()
    buf = ScanBuffer()
    node = ScanNode(args.topic, buf)

    # Runs in headless mode: prints sector classification periodically
    if args.no_gui:
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                msg, _ = buf.get()
                if msg is None:
                    continue
                ang, rng, *_ = scan_to_xy(msg, args.max_range)
                label, mins, _ = classify_sector(
                    ang, rng,
                    front_deg=args.front_deg,
                    side_deg=args.side_deg,
                    margin_deg=args.margin_deg,
                    range_min=msg.range_min,
                    range_max=msg.range_max,
                )
                print(f"{time.strftime('%H:%M:%S')} nearest={label:8s}  "
                      f"front={mins['front']:.2f}  left={mins['left']:.2f}  "
                      f"right={mins['right']:.2f}  between={mins['between']:.2f}")
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    # Sets up an interactive Matplotlib GUI to visualize points and sectors
    plt.ion()
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, aspect='equal')
    ax.set_title("LiDAR (XY in laser_link)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, linestyle=':', linewidth=0.5)

    # Sets plot limits based on desired display range
    disp_range = args.max_range or 10.0
    ax.set_xlim(-disp_range, disp_range)
    ax.set_ylim(-disp_range, disp_range)

    # Adds colored wedges to show front/left/right sectors
    def add_wedge(start_deg, end_deg, color, alpha=0.08):
        # Matplotlib wedge angles rotate CCW from +X axis
        w = Wedge(center=(0, 0), r=disp_range, theta1=start_deg, theta2=end_deg,
                  facecolor=color, alpha=alpha, edgecolor='none')
        ax.add_patch(w)
        return w

    # Draws sector wedges using configured angles
    front_w = add_wedge(-args.front_deg, +args.front_deg, 'tab:green')
    left_w  = add_wedge(args.front_deg + args.margin_deg, args.side_deg, 'tab:blue')
    right_w = add_wedge(-args.side_deg, -(args.front_deg + args.margin_deg), 'tab:orange')

    # Creates a scatter for valid scan points and markers for per-sector closest points
    scat = ax.scatter([], [], s=6, c='k', alpha=0.8)
    markers = {
        "front":   ax.plot([], [], 'o', color='tab:green', markersize=8)[0],
        "left":    ax.plot([], [], 'o', color='tab:blue',  markersize=8)[0],
        "right":   ax.plot([], [], 'o', color='tab:orange',markersize=8)[0],
        "between": ax.plot([], [], 'o', color='tab:gray',  markersize=8)[0],
    }

    # Adds text overlays for nearest sector and per-sector distances
    text_near = ax.text(0.02, 0.98, "", transform=ax.transAxes, va='top', ha='left',
                        bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
    text_vals = ax.text(0.02, 0.90, "", transform=ax.transAxes, va='top', ha='left',
                        bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

    paused = {"v": False}

    def on_key(event):
        # Handles quick controls: q=quit, p=pause, s=screenshot
        if event.key == 'q':
            plt.close(fig)
        elif event.key == 'p':
            paused["v"] = not paused["v"]
        elif event.key == 's':
            fname = f"lidar_{int(time.time())}.png"
            fig.savefig(fname, dpi=150)
            print(f"saved {fname}")

    fig.canvas.mpl_connect('key_press_event', on_key)

    last_ns = 0
    try:
        # Runs the UI loop: consumes scans, classifies sectors, and updates the plot
        while plt.fignum_exists(fig.number) and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if paused["v"]:
                plt.pause(1.0 / args.rate)
                continue

            msg, stamp_ns = buf.get()
            if msg is None or stamp_ns == last_ns:
                plt.pause(0.01)
                continue
            last_ns = stamp_ns

            angles, ranges, xs, ys = scan_to_xy(msg, args.max_range)

            # Updates scatter with finite points only
            finite = np.isfinite(ranges)
            scat.set_offsets(np.c_[xs[finite], ys[finite]])

            # Classifies sectors and finds closest points per sector
            label, mins, idxs = classify_sector(
                angles, ranges,
                front_deg=args.front_deg,
                side_deg=args.side_deg,
                margin_deg=args.margin_deg,
                range_min=msg.range_min,
                range_max=msg.range_max,
            )

            # Updates per-sector “closest” markers
            for name, idx in idxs.items():
                if idx is None or not math.isfinite(ranges[idx]):
                    markers[name].set_data([], [])
                else:
                    markers[name].set_data([xs[idx]], [ys[idx]])

            # Updates overlay text with current classification and distances
            text_near.set_text(f"NEAREST: {label.upper()}  (stamp {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d})")
            text_vals.set_text(
                "front:   {:.2f} m\nleft:    {:.2f} m\nright:   {:.2f} m\nbetween: {:.2f} m".format(
                    mins["front"], mins["left"], mins["right"], mins["between"]
                )
            )

            # Draws the frame at the chosen refresh rate
            plt.pause(1.0 / args.rate)

    except KeyboardInterrupt:
        pass
    finally:
        # Cleans up the ROS node and shuts down gracefully
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
