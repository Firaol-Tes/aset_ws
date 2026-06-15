#!/usr/bin/env python3
"""
perception_node.py  —  HSV color detector (primary) with YOLOv8n fallback.

Synchronises RGB + depth images, back-projects to 3-D via PinholeCameraModel,
transforms to the map frame with TF2, and publishes DetectionArray.

Parameters:
  detector              color | yolo   (default: color)
  confidence_threshold  float          (default: 0.45, used by YOLO mode)
  min_contour_area      int            (default: 200 pixels)
"""

import numpy as np
import rclpy
from rclpy.node import Node
import rclpy.duration
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  — registers the PointStamped do_transform
import cv2

from mm_interfaces.msg import Detection, DetectionArray


# HSV ranges for each cube colour.
# Each entry is a list of (lower, upper) pairs (some colours need two ranges for hue wrap-around).
_HSV_RANGES = {
    'red_cube': [
        (np.array([0,   120,  70]), np.array([10,  255, 255])),
        (np.array([170, 120,  70]), np.array([180, 255, 255])),
    ],
    'green_cube': [
        (np.array([40,  80,  50]), np.array([85,  255, 255])),
    ],
    'blue_cube': [
        (np.array([100, 80,  50]), np.array([140, 255, 255])),
    ],
}

_MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('detector', 'color')
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('min_contour_area', 200)

        self._bridge = CvBridge()
        self._cam_model = PinholeCameraModel()
        self._cam_info_received = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Camera info: rarely changes — use a latching-style single subscriber
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self._camera_info_cb, rclpy.qos.qos_profile_sensor_data)

        # Synchronised RGB + depth
        rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/image_raw',
            qos_profile=rclpy.qos.qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(
            self, Image, '/camera/depth',
            qos_profile=rclpy.qos.qos_profile_sensor_data)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=5, slop=0.15)
        self._sync.registerCallback(self._image_cb)

        self._pub = self.create_publisher(DetectionArray, '/detected_objects', 10)

        # YOLO model loaded lazily
        self._yolo_model = None

        self.get_logger().info('Perception node ready (detector=%s)' %
                               self.get_parameter('detector').value)

    # ── camera info ────────────────────────────────────────────────────────────

    def _camera_info_cb(self, msg: CameraInfo):
        if not self._cam_info_received:
            self._cam_model.fromCameraInfo(msg)
            self._cam_info_received = True
            self.get_logger().info(
                'Camera model set: %dx%d  fx=%.1f fy=%.1f' %
                (msg.width, msg.height, msg.k[0], msg.k[4]))

    # ── main image callback ────────────────────────────────────────────────────

    def _image_cb(self, rgb_msg: Image, depth_msg: Image):
        if not self._cam_info_received:
            return

        detector = self.get_parameter('detector').value

        bgr   = self._bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self._bridge.imgmsg_to_cv2(depth_msg, '32FC1')

        if detector == 'yolo':
            dets = self._detect_yolo(bgr, depth, rgb_msg.header)
        else:
            dets = self._detect_color(bgr, depth, rgb_msg.header)

        arr = DetectionArray()
        arr.header = rgb_msg.header
        arr.detections = dets
        self._pub.publish(arr)

    # ── HSV colour detector ────────────────────────────────────────────────────

    def _detect_color(self, bgr, depth, header) -> list:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        min_area = self.get_parameter('min_contour_area').value
        img_pixels = bgr.shape[0] * bgr.shape[1]
        detections = []

        for label, ranges in _HSV_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, lo, hi)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  _MORPH_KERNEL)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH_KERNEL)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < min_area:
                    continue

                M = cv2.moments(cnt)
                if M['m00'] == 0:
                    continue
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                pt3d = self._pixel_to_map(cx, cy, depth, header)
                if pt3d is None:
                    continue

                d = Detection()
                d.label      = label
                d.color      = label.split('_')[0]
                d.x, d.y, d.z = pt3d
                d.confidence = min(float(area) / img_pixels * 100.0, 1.0)
                detections.append(d)

        return detections

    # ── YOLOv8 detector ───────────────────────────────────────────────────────

    def _detect_yolo(self, bgr, depth, header) -> list:
        if self._yolo_model is None:
            import sys
            # Ultralytics lives in the venv; add it only when needed
            venv_sp = '/home/f/aset_ws/venv/lib/python3.12/site-packages'
            if venv_sp not in sys.path:
                sys.path.insert(0, venv_sp)
            from ultralytics import YOLO
            self._yolo_model = YOLO('yolov8n.pt')
            self.get_logger().info('YOLOv8n model loaded')

        conf_thresh = self.get_parameter('confidence_threshold').value
        results = self._yolo_model(bgr, verbose=False)
        detections = []

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < conf_thresh:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                pt3d = self._pixel_to_map(cx, cy, depth, header)
                if pt3d is None:
                    continue

                cls_id = int(box.cls[0])
                label  = self._yolo_model.names[cls_id]

                d = Detection()
                d.label      = label
                d.color      = ''
                d.x, d.y, d.z = pt3d
                d.confidence = conf
                detections.append(d)

        return detections

    # ── depth back-projection + TF ────────────────────────────────────────────

    def _pixel_to_map(self, cx: int, cy: int, depth, header):
        """Back-project (cx,cy) through depth to a 3-D point in the map frame.
        Returns (x, y, z) tuple or None on failure."""
        h, w = depth.shape
        if cx < 0 or cx >= w or cy < 0 or cy >= h:
            return None

        z = float(depth[cy, cx])
        if np.isnan(z) or z <= 0.01 or z > 10.0:
            return None

        # PinholeCameraModel gives a ray s.t. ray[2]==1 in the optical frame
        ray = self._cam_model.projectPixelTo3dRay((cx, cy))
        pt_cam = PointStamped()
        pt_cam.header = header
        pt_cam.point.x = ray[0] * z
        pt_cam.point.y = ray[1] * z
        pt_cam.point.z = ray[2] * z  # == z for a normalised ray

        # Try map frame first, fall back to odom if SLAM isn't running
        for target_frame in ('map', 'odom'):
            try:
                pt_out = self._tf_buffer.transform(
                    pt_cam, target_frame,
                    timeout=rclpy.duration.Duration(seconds=0.1))
                return (pt_out.point.x, pt_out.point.y, pt_out.point.z)
            except Exception:
                pass

        return None


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
