#!/usr/bin/env python3
import argparse, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

def img_to_rgb(bridge, msg):
    # Converts a ROS Image to RGB8 (handles RGB/BGR encodings)
    enc = (msg.encoding or "").lower()
    if enc.startswith('rgb'):
        return bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    return bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')[:, :, ::-1]

def img_to_depth_m(bridge, msg):
    # Converts a ROS depth image to float meters (supports 16U mm and 32FC1)
    enc = (msg.encoding or "").lower()
    arr = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    if '16u' in enc:   # mm -> m
        return arr.astype(np.float32) * 0.001
    return arr.astype(np.float32)  # 32FC1 already meters

class RGBDIRViewer(Node):
    def __init__(self, color_t, depth_t, ir_t, info_t,
                 dmin, dmax, step, max_pts,
                 show_rgb, show_depth_image, show_ir, show_pcd):
        super().__init__('rgbdir_mpl_viewer')

        # Stores which panels the UI displays
        self.show_rgb = show_rgb
        self.show_depth_image = show_depth_image
        self.show_ir = show_ir
        self.show_pcd = show_pcd

        # Initializes frame buffers and camera intrinsics
        self.b = CvBridge()
        self.rgb = None
        self.ir  = None
        self.depth = None
        self.K = None
        self.dmin, self.dmax = dmin, dmax
        self.step = max(1, step)
        self.max_pts = max_pts

        # Subscribes only to the streams we actually need
        if self.show_rgb:
            self.create_subscription(Image, color_t, self._on_color, qos_profile_sensor_data)
        if self.show_ir:
            self.create_subscription(Image, ir_t,    self._on_ir,    qos_profile_sensor_data)
        if self.show_depth_image or self.show_pcd:
            self.create_subscription(Image, depth_t, self._on_depth, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, info_t, self._on_info, qos_profile_sensor_data)

        # Builds a dynamic 1×N Matplotlib layout (RGB | PCD | IR | Depth)
        panels = []
        if self.show_rgb:        panels.append('rgb')
        if self.show_pcd:        panels.append('pcd')
        if self.show_ir:         panels.append('ir')
        if self.show_depth_image:panels.append('depth_img')

        if not panels:
            raise RuntimeError("All panels disabled. Enable at least one of --show-rgb/--show-pointcloud/--show-ir/--show-depth-image.")

        plt.ion()
        self.fig = plt.figure(figsize=(5 * len(panels), 5))
        self.axes = {}
        for i, key in enumerate(panels, 1):
            if key == 'pcd':
                ax = self.fig.add_subplot(1, len(panels), i, projection='3d')
                ax.set_title('Depth Point Cloud (m)')
            elif key == 'depth_img':
                ax = self.fig.add_subplot(1, len(panels), i)
                ax.set_title('Depth (m)')
                ax.axis('off')
            elif key == 'rgb':
                ax = self.fig.add_subplot(1, len(panels), i)
                ax.set_title('RGB')
                ax.axis('off')
            elif key == 'ir':
                ax = self.fig.add_subplot(1, len(panels), i)
                ax.set_title('IR')
                ax.axis('off')
            self.axes[key] = ax

        # Keeps handles to Matplotlib images for fast updates
        self.im_handles = {'rgb': None, 'ir': None, 'depth_img': None}

        # Logs once when the first frame for each stream arrives
        self.seen = {'rgb': False, 'ir': False, 'depth': False}

    # ---- Callbacks
    def _on_color(self, msg: Image):
        # Converts and stores the latest RGB frame
        self.rgb = img_to_rgb(self.b, msg)
        if not self.seen['rgb']:
            self.get_logger().info('RGB: first frame')
            self.seen['rgb'] = True

    def _on_depth(self, msg: Image):
        # Converts and stores the latest depth frame (meters), masking nonpositive as NaN
        d = img_to_depth_m(self.b, msg)
        d[d <= 0] = np.nan
        self.depth = d
        if not self.seen['depth']:
            self.get_logger().info(f'Depth: first frame (encoding={msg.encoding})')
            self.seen['depth'] = True

    def _on_ir(self, msg: Image):
        # Converts and stores the latest IR frame as 8-bit grayscale
        enc = (msg.encoding or "").lower()
        if 'mono16' in enc:
            im = self.b.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            im = np.clip(im / 256.0, 0, 255).astype(np.uint8)
        else:
            im = self.b.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self.ir = im
        if not self.seen['ir']:
            self.get_logger().info('IR: first frame')
            self.seen['ir'] = True

    def _on_info(self, msg: CameraInfo):
        # Stores the camera intrinsic matrix K for point cloud projection
        self.K = np.array(msg.k, dtype=np.float32).reshape(3, 3)

    # ---- Updates (called from main loop)
    def update_rgb(self):
        # Updates the RGB panel if enabled and a frame is available
        if not self.show_rgb or self.rgb is None: return
        ax = self.axes['rgb']
        if self.im_handles['rgb'] is None:
            self.im_handles['rgb'] = ax.imshow(self.rgb)
        else:
            self.im_handles['rgb'].set_data(self.rgb)

    def update_ir(self):
        # Updates the IR panel if enabled and a frame is available
        if not self.show_ir or self.ir is None: return
        ax = self.axes['ir']
        if self.im_handles['ir'] is None:
            self.im_handles['ir'] = ax.imshow(self.ir, cmap='gray')
        else:
            self.im_handles['ir'].set_data(self.ir)

    def update_depth_image(self):
        # Updates the depth image panel with colorbar scaling
        if not self.show_depth_image or self.depth is None: return
        ax = self.axes['depth_img']
        if self.im_handles['depth_img'] is None:
            self.im_handles['depth_img'] = ax.imshow(self.depth, cmap='plasma',
                                                     vmin=self.dmin, vmax=self.dmax)
            self.fig.colorbar(self.im_handles['depth_img'], ax=ax, fraction=0.046, pad=0.04)
        else:
            self.im_handles['depth_img'].set_data(self.depth)
            self.im_handles['depth_img'].set_clim(vmin=self.dmin, vmax=self.dmax)

    def update_pcd(self):
        # Reprojects the depth image into a 3D point cloud and refreshes the plot
        if not self.show_pcd or self.depth is None or self.K is None:
            return

        d = self.depth
        h, w = d.shape
        fx, fy = float(self.K[0, 0]), float(self.K[1, 1])
        cx, cy = float(self.K[0, 2]), float(self.K[1, 2])

        xs = np.arange(0, w, self.step)
        ys = np.arange(0, h, self.step)
        xv, yv = np.meshgrid(xs, ys)
        z_fwd = d[yv, xv]                       # Depth forward (meters)
        m = ~np.isnan(z_fwd)
        if not np.any(m):
            return

        x_right = (xv[m] - cx) * z_fwd[m] / fx  # Camera X: right
        y_up    = -(yv[m] - cy) * z_fwd[m] / fy # Camera Z: up (flip so + is up)
        z_fwd   = z_fwd[m]                      # Camera Y: forward

        # Caps the number of plotted points for speed
        if x_right.size > self.max_pts:
            idx = np.random.choice(x_right.size, self.max_pts, replace=False)
            x_right, y_up, z_fwd = x_right[idx], y_up[idx], z_fwd[idx]

        ax = self.axes['pcd']
        ax.cla()
        ax.set_title('Depth Point Cloud (m)')
        ax.set_xlabel('X (right)')
        ax.set_ylabel('Y (forward)')
        ax.set_zlabel('Z (up)')
        ax.view_init(elev=20, azim=-60)

        s = 0.5 if x_right.size > 50000 else 1.5
        ax.scatter(x_right, z_fwd, y_up,  # Plots as (X, Y=forward, Z=up)
                   c=z_fwd, cmap='viridis', s=s, depthshade=False)

        # Sets reasonable Z (up) limits to keep the view stable
        if np.isfinite(y_up).any():
            zmin = float(np.nanpercentile(y_up, 1))
            zmax = float(np.nanpercentile(y_up, 99))
            ax.set_zlim(zmin, zmax)

def main():
    ap = argparse.ArgumentParser(description='Live: RGB | Depth Point Cloud | IR | (optional Depth image)')
    # Topics
    ap.add_argument('--color', default='/orbbec/color/image_raw')
    ap.add_argument('--depth', default='/orbbec/depth/image_raw')
    ap.add_argument('--ir',    default='/orbbec/ir/image_raw')
    ap.add_argument('--info',  default='/orbbec/depth/camera_info')
    # Visualization params
    ap.add_argument('--dmin', type=float, default=0.3)
    ap.add_argument('--dmax', type=float, default=3.0)
    ap.add_argument('--step', type=int,   default=6)
    ap.add_argument('--max-pts', type=int, default=60000)
    # Toggles (CLI-friendly on/off switches)
    ap.add_argument('--show-rgb',          action='store_true',  default=True)
    ap.add_argument('--no-show-rgb',       dest='show_rgb',      action='store_false')
    ap.add_argument('--show-depth-image',  action='store_true',  default=True)
    ap.add_argument('--no-show-depth-image', dest='show_depth_image', action='store_false')
    ap.add_argument('--show-ir',           action='store_true',  default=True)
    ap.add_argument('--no-show-ir',        dest='show_ir',       action='store_false')
    ap.add_argument('--show-pointcloud',   action='store_true',  default=True)
    ap.add_argument('--no-show-pointcloud', dest='show_pointcloud', action='store_false')

    args = ap.parse_args()

    rclpy.init()
    node = RGBDIRViewer(
        args.color, args.depth, args.ir, args.info,
        args.dmin, args.dmax, args.step, args.max_pts,
        show_rgb=args.show_rgb,
        show_depth_image=args.show_depth_image,
        show_ir=args.show_ir,
        show_pcd=args.show_pointcloud
    )

    # Runs ROS callbacks on a background thread while the UI updates in the main thread
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    ros_thread = threading.Thread(target=exec_.spin, daemon=True)
    ros_thread.start()

    try:
        # Steps each panel, refreshes the figure, and throttles a bit
        while rclpy.ok():
            node.update_rgb()
            node.update_pcd()
            node.update_ir()
            node.update_depth_image()
            plt.tight_layout()
            plt.pause(0.01)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        # Shuts down the executor, the node, ROS, and closes plots
        exec_.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()
