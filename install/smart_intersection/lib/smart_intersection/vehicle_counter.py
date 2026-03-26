#!/usr/bin/env python3
"""
vehicle_counter.py — Enhanced CV-based traffic analyzer
========================================================
Analyzes each camera feed to produce:
  - Vehicle count per direction
  - Traffic density classification (LOW/MEDIUM/HIGH/VERY_HIGH)
  - Queue length estimation
  - Vehicle type classification (size-based)
  - Emergency vehicle detection (HSV colour analysis)

Publishes:
  /vehicle_counts  — backward-compatible "DIR:COUNT" per direction
  /traffic_analysis — JSON with full analysis per direction
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

try:
    import cv2
    import numpy as np
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

# Mapping from camera topic to direction label
CAM_TOPICS = {
    '/cam_north/image_raw': 'N',
    '/cam_east/image_raw':  'E',
    '/cam_south/image_raw': 'S',
    '/cam_west/image_raw':  'W',
}

# Contour area thresholds for vehicle type classification
AREA_BIKE  = 300     # min area for bike
AREA_CAR   = 800     # min area for car
AREA_BUS   = 2500    # min area for bus/truck

# Density classification thresholds
DENSITY_THRESHOLDS = {
    'LOW':       (0, 2),
    'MEDIUM':    (3, 5),
    'HIGH':      (6, 9),
    'VERY_HIGH': (10, 999),
}


class VehicleCounter(Node):
    def __init__(self):
        super().__init__('vehicle_counter')

        self.bridge = None
        self.cv_available = CV_AVAILABLE

        if self.cv_available:
            try:
                from cv_bridge import CvBridge
                self.bridge = CvBridge()
            except ImportError:
                self.cv_available = False

        # Background subtractors per direction
        self.subtractors = {
            d: cv2.createBackgroundSubtractorMOG2(
                history=500, varThreshold=50, detectShadows=True
            )
            for d in CAM_TOPICS.values()
        } if self.cv_available else {}

        # Publishers
        self.count_pub    = self.create_publisher(String, '/vehicle_counts', 10)
        self.analysis_pub = self.create_publisher(String, '/traffic_analysis', 10)

        # Subscribe to all camera topics
        for topic in CAM_TOPICS:
            self.create_subscription(
                Image, topic,
                lambda msg, t=topic: self.image_cb(msg, t), 10
            )

        # Store latest analysis per direction
        self.latest_analysis = {d: self._empty_analysis(d) for d in CAM_TOPICS.values()}

        self.get_logger().info(
            f'VehicleCounter READY (CV available: {self.cv_available})'
        )

    def _empty_analysis(self, direction):
        return {
            'direction': direction,
            'vehicle_count': 0,
            'density': 'LOW',
            'queue_length': 0,
            'vehicle_types': {'bike': 0, 'car': 0, 'bus_truck': 0},
            'emergency_detected': False,
        }

    def _classify_density(self, count):
        for level, (low, high) in DENSITY_THRESHOLDS.items():
            if low <= count <= high:
                return level
        return 'VERY_HIGH'

    def _detect_emergency_hsv(self, hsv_frame):
        """Detect emergency vehicles by HSV colour analysis."""
        # Ambulance: white body + red cross
        white_mask = cv2.inRange(hsv_frame, np.array([0, 0, 200]), np.array([180, 50, 255]))
        red_mask1 = cv2.inRange(hsv_frame, np.array([0, 150, 100]), np.array([10, 255, 255]))
        red_mask2 = cv2.inRange(hsv_frame, np.array([170, 150, 100]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        if cv2.countNonZero(white_mask) > 500 and cv2.countNonZero(red_mask) > 100:
            return True

        # Fire engine: large bright red area
        fire_mask = cv2.inRange(hsv_frame, np.array([0, 150, 100]), np.array([10, 255, 255]))
        contours, _ = cv2.findContours(fire_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > 800:
                return True

        # Police: blue body
        blue_mask = cv2.inRange(hsv_frame, np.array([100, 100, 50]), np.array([130, 255, 255]))
        if cv2.countNonZero(blue_mask) > 600:
            return True

        return False

    def _classify_vehicles(self, contours):
        """Classify detected contours into vehicle types by area."""
        types = {'bike': 0, 'car': 0, 'bus_truck': 0}
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= AREA_BUS:
                types['bus_truck'] += 1
            elif area >= AREA_CAR:
                types['car'] += 1
            elif area >= AREA_BIKE:
                types['bike'] += 1
        return types

    def _estimate_queue_length(self, contours, frame_height):
        """Estimate queue length from vertical distribution of contours."""
        if not contours:
            return 0
        # Count vehicles in the lower 2/3 of frame (approaching area)
        queue = 0
        threshold_y = frame_height * 0.33
        for cnt in contours:
            M = cv2.moments(cnt)
            if M['m00'] > 0:
                cy = M['m01'] / M['m00']
                if cy > threshold_y:  # in approaching zone
                    queue += 1
        return queue

    def image_cb(self, msg: Image, topic: str):
        direction = CAM_TOPICS.get(topic)
        if direction is None:
            return

        if not self.cv_available or self.bridge is None:
            analysis = self._empty_analysis(direction)
            self._publish_results(direction, analysis)
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Background subtraction
        fgmask = self.subtractors[direction].apply(frame)
        _, thresh = cv2.threshold(fgmask, 200, 255, cv2.THRESH_BINARY)

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter small noise contours
        valid_contours = [c for c in contours if cv2.contourArea(c) > AREA_BIKE]

        # Count and classify
        vehicle_count = len(valid_contours)
        vehicle_types = self._classify_vehicles(valid_contours)
        queue_length = self._estimate_queue_length(valid_contours, frame.shape[0])

        # Emergency detection via HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        emergency = self._detect_emergency_hsv(hsv)

        # Build analysis
        analysis = {
            'direction': direction,
            'vehicle_count': vehicle_count,
            'density': self._classify_density(vehicle_count),
            'queue_length': queue_length,
            'vehicle_types': vehicle_types,
            'emergency_detected': emergency,
        }

        self.latest_analysis[direction] = analysis
        self._publish_results(direction, analysis)

    def _publish_results(self, direction, analysis):
        # Backward-compatible count
        count_msg = String()
        count_msg.data = f"{direction}:{analysis['vehicle_count']}"
        self.count_pub.publish(count_msg)

        # Full analysis as JSON
        analysis_msg = String()
        analysis_msg.data = json.dumps(analysis)
        self.analysis_pub.publish(analysis_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleCounter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()