#!/usr/bin/env python3
"""Phase II — Real-time object detection from the robot's cameras.

Subscribes to mono camera + depth camera, publishes annotated image,
3D marker array, and PoseStamped for each detected object.

HSV calibration mode:
  ros2 run sim_gazebo_bringup object_detector --ros-args -p calibrate_mode:=true
"""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA, Header

try:
    from cv_bridge import CvBridge
    import cv2
    import numpy as np
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


def _depth_at(depth_img, u, v, radius=3):
    """Median depth in a square patch around (u,v), or 0 if invalid."""
    h, w = depth_img.shape[:2]
    if not (0 <= u < w and 0 <= v < h):
        return 0.0
    y0, y1 = max(0, v - radius), min(h, v + radius + 1)
    x0, x1 = max(0, u - radius), min(w, u + radius + 1)
    patch = depth_img[y0:y1, x0:x1]
    valid = patch[~np.isnan(patch) & (patch > 0)]
    return float(np.median(valid)) if len(valid) > 0 else 0.0


def _project(cv_img, center_u, center_v, depth_val, kinfo):
    """Project pixel + depth to 3D point in camera frame using camera intrinsics."""
    k = kinfo.k
    fx, fy = k[0], k[4]
    cx, cy = k[2], k[5]
    if fx == 0 or fy == 0 or depth_val <= 0:
        return None
    z = float(depth_val)
    x = (float(center_u) - cx) * z / fx
    y = (float(center_v) - cy) * z / fy
    pt = Point(x=x, y=y, z=z)
    cv2.circle(cv_img, (center_u, center_v), 4, (0, 255, 255), -1)
    cv2.putText(cv_img, f'{z:.2f}m', (center_u + 6, center_v + 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    return pt


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        if not CV_AVAILABLE:
            self.get_logger().error('cv_bridge / cv2 not available')
            return

        self._bridge = CvBridge()

        # Parameters
        self.declare_parameter('camera_topic', '/mono_camera/image_raw')
        self.declare_parameter('depth_topic', '/depth_camera/depth_image')
        self.declare_parameter('camera_info_topic', '/depth_camera/camera_info')
        # Default HSV tuned for the blue/cyan test_block in Gazebo.
        # The block ambient is RGB(0, 0.5, 1.0) → BGR(255, 128, 0).
        # In OpenCV HSV this sits around hue 90–110 (0-179 scale).
        self.declare_parameter('hsv_lower_h', 80)
        self.declare_parameter('hsv_lower_s', 50)
        self.declare_parameter('hsv_lower_v', 50)
        self.declare_parameter('hsv_upper_h', 120)
        self.declare_parameter('hsv_upper_s', 255)
        self.declare_parameter('hsv_upper_v', 255)
        self.declare_parameter('min_area', 500)
        self.declare_parameter('max_objects', 5)
        self.declare_parameter('calibrate_mode', False)
        self.declare_parameter('depth_scale', 0.001)

        cam_topic = self.get_parameter('camera_topic').value
        dep_topic = self.get_parameter('depth_topic').value
        ci_topic = self.get_parameter('camera_info_topic').value

        self._sub = self.create_subscription(Image, cam_topic, self._on_image, 1)
        self._depth_sub = self.create_subscription(Image, dep_topic, self._on_depth, 1)
        self._cinfo_sub = self.create_subscription(CameraInfo, ci_topic, self._on_camera_info, 1)

        self._pub_img = self.create_publisher(Image, '/detected_image', 1)
        self._pub_markers = self.create_publisher(MarkerArray, '/detected_objects', 10)
        self._pub_pose = self.create_publisher(PoseStamped, '/detected_object_pose', 10)

        self._latest_depth = None
        self._camera_info = None
        self._calibrate_mode = self.get_parameter('calibrate_mode').value

        self.get_logger().info(
            f'Listening on {cam_topic} + {dep_topic}, publishing to '
            '/detected_image, /detected_objects, /detected_object_pose')

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_depth(self, msg: Image) -> None:
        try:
            self._latest_depth = self._bridge.imgmsg_to_cv2(msg, '32FC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')

    def _hsv_params(self):
        g = self.get_parameter
        return (np.array([g('hsv_lower_h').value, g('hsv_lower_s').value, g('hsv_lower_v').value]),
                np.array([g('hsv_upper_h').value, g('hsv_upper_s').value, g('hsv_upper_v').value]))

    def _on_image(self, msg: Image) -> None:
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return

        lower, upper = self._hsv_params()
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_area = self.get_parameter('min_area').value
        max_obj = self.get_parameter('max_objects').value

        detections = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cu, cv_ = x + w // 2, y + h // 2
            depth = 0.0
            if self._latest_depth is not None:
                depth = _depth_at(self._latest_depth, cu, cv_)
            pt3d = None
            if depth > 0 and self._camera_info is not None:
                pt3d = _project(cv_img, cu, cv_, depth, self._camera_info)
            detections.append(((x, y, w, h), (cu, cv_), depth, pt3d, area))

        detections.sort(key=lambda d: d[4], reverse=True)
        detections = detections[:max_obj]

        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        frame_id = self._camera_info.header.frame_id if self._camera_info else 'camera_link'

        for i, det in enumerate(detections):
            (x, y, w, h), (cu, cv_), depth, pt3d, area = det
            t = min(depth / 5.0, 1.0) if depth > 0 else 0
            color = (0, int(255 * t), int(255 * (1 - t)))
            cv2.rectangle(cv_img, (x, y), (x + w, y + h), color, 2)
            label = f'obj{i}'
            if depth > 0:
                label += f' {depth:.2f}m'
            cv2.putText(cv_img, label, (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            if pt3d is not None:
                m = Marker()
                m.header = Header(stamp=now, frame_id=frame_id)
                m.ns = 'detected_objects'
                m.id = i
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position = pt3d
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.08
                m.color = ColorRGBA(r=1.0, g=0.2, b=0.0, a=0.8)
                m.lifetime.sec = 1
                markers.markers.append(m)

                pose = PoseStamped()
                pose.header = Header(stamp=now, frame_id=frame_id)
                pose.pose.position = pt3d
                pose.pose.orientation.w = 1.0
                self._pub_pose.publish(pose)

        if markers.markers:
            self._pub_markers.publish(markers)

        try:
            out = self._bridge.cv2_to_imgmsg(cv_img, 'bgr8')
            out.header = msg.header
            self._pub_img.publish(out)
        except Exception as e:
            self.get_logger().warn(f'cv2_to_imgmsg failed: {e}')

        if self._calibrate_mode and self._camera_info is not None:
            self._show_calibrator(cv_img, mask)

    _calibrator_initialized = False

    def _show_calibrator(self, bgr, mask):
        if not self._calibrator_initialized:
            self._calibrator_initialized = True
            cv2.namedWindow('HSV Calibrator')
            cv2.resizeWindow('HSV Calibrator', 640, 480)
            cv2.createTrackbar('H Low', 'HSV Calibrator', self.get_parameter('hsv_lower_h').value, 180, lambda v: None)
            cv2.createTrackbar('H High', 'HSV Calibrator', self.get_parameter('hsv_upper_h').value, 180, lambda v: None)
            cv2.createTrackbar('S Low', 'HSV Calibrator', self.get_parameter('hsv_lower_s').value, 255, lambda v: None)
            cv2.createTrackbar('S High', 'HSV Calibrator', self.get_parameter('hsv_upper_s').value, 255, lambda v: None)
            cv2.createTrackbar('V Low', 'HSV Calibrator', self.get_parameter('hsv_lower_v').value, 255, lambda v: None)
            cv2.createTrackbar('V High', 'HSV Calibrator', self.get_parameter('hsv_upper_v').value, 255, lambda v: None)
        h_low = cv2.getTrackbarPos('H Low', 'HSV Calibrator')
        h_high = cv2.getTrackbarPos('H High', 'HSV Calibrator')
        s_low = cv2.getTrackbarPos('S Low', 'HSV Calibrator')
        s_high = cv2.getTrackbarPos('S High', 'HSV Calibrator')
        v_low = cv2.getTrackbarPos('V Low', 'HSV Calibrator')
        v_high = cv2.getTrackbarPos('V High', 'HSV Calibrator')
        self.set_parameters([
            rclpy.parameter.Parameter('hsv_lower_h', rclpy.parameter.Parameter.Type.INTEGER, h_low),
            rclpy.parameter.Parameter('hsv_upper_h', rclpy.parameter.Parameter.Type.INTEGER, h_high),
            rclpy.parameter.Parameter('hsv_lower_s', rclpy.parameter.Parameter.Type.INTEGER, s_low),
            rclpy.parameter.Parameter('hsv_upper_s', rclpy.parameter.Parameter.Type.INTEGER, s_high),
            rclpy.parameter.Parameter('hsv_lower_v', rclpy.parameter.Parameter.Type.INTEGER, v_low),
            rclpy.parameter.Parameter('hsv_upper_v', rclpy.parameter.Parameter.Type.INTEGER, v_high),
        ])
        overlay = cv2.resize(bgr, (320, 240))
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_small = cv2.resize(mask_bgr, (320, 240))
        combined = np.vstack([overlay, mask_small])
        cv2.imshow('HSV Calibrator', combined)
        cv2.waitKey(1)


def main():
    if not CV_AVAILABLE:
        print('ERROR: OpenCV / cv_bridge not available.')
        sys.exit(1)
    rclpy.init()
    node = ObjectDetector()
    if not CV_AVAILABLE or node._bridge is None:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
