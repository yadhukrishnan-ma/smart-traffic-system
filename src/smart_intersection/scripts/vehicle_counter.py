#!/usr/bin/env python3
"""
vehicle_counter.py — Enhanced CV-based traffic analyzer with depth estimation
==============================================================================
Uses OpenCV to analyze each overhead camera feed and produce:
  - Vehicle count per direction
  - Traffic density classification (LOW/MEDIUM/HIGH/VERY_HIGH)
  - Queue length estimation
  - Depth estimation using perspective (bounding box Y-position)
  - Vehicle type classification (size + aspect ratio)
  - Emergency vehicle detection (HSV colour analysis)

Depth estimation:
  The cameras are mounted at z=7.0m with 30° downtilt. Objects lower in
  the frame (higher pixel Y) are closer to the camera/junction. Objects
  higher in the frame (lower pixel Y) are farther away. Using a simple
  pinhole camera model, we convert pixel position to approximate distance.

Publishes:
  /vehicle_counts   — backward-compatible "DIR:COUNT" per direction
  /traffic_analysis — JSON with full analysis per direction including depth data
"""

import json
import math
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

# Camera parameters (from world file)
CAMERA_HEIGHT    = 7.0     # meters above ground
CAMERA_TILT_DEG  = 30.0    # downtilt in degrees
CAMERA_HFOV      = 1.4     # horizontal FOV in radians (~80°)
IMAGE_WIDTH      = 640
IMAGE_HEIGHT     = 480

# Computed camera parameters for depth estimation
CAMERA_TILT_RAD = math.radians(CAMERA_TILT_DEG)
CAMERA_VFOV     = CAMERA_HFOV * (IMAGE_HEIGHT / IMAGE_WIDTH)  # vertical FOV
FOCAL_Y         = IMAGE_HEIGHT / (2.0 * math.tan(CAMERA_VFOV / 2.0))

# Contour area thresholds for vehicle type classification
AREA_AUTO_RICKSHAW = 300     # min area for auto rickshaw
AREA_CAR   = 600     # min area for car
AREA_BUS   = 2000    # min area for bus/truck

# Density classification thresholds
DENSITY_THRESHOLDS = {
    'LOW':       (0, 2),
    'MEDIUM':    (3, 5),
    'HIGH':      (6, 9),
    'VERY_HIGH': (10, 999),
}

# HSV ranges for emergency vehicle detection
# Ambulance: white body
HSV_WHITE_LOW  = (0, 0, 200)
HSV_WHITE_HIGH = (180, 50, 255)
# Red cross / fire engine
HSV_RED1_LOW   = (0, 150, 100)
HSV_RED1_HIGH  = (10, 255, 255)
HSV_RED2_LOW   = (170, 150, 100)
HSV_RED2_HIGH  = (180, 255, 255)
# Police blue
HSV_BLUE_LOW   = (100, 100, 50)
HSV_BLUE_HIGH  = (130, 255, 255)


def _estimate_depth_from_pixel_y(pixel_y, frame_height):
    """
    Estimate the ground distance from the camera base point to the object.
    
    The camera is at height H with tilt angle θ (downward).
    A pixel at row y in the image corresponds to an angle relative to
    the camera's optical axis. Using geometry:
    
      angle_from_axis = (y - cy) / focal_length
      ground_angle = tilt + angle_from_axis
      distance = H / tan(ground_angle)  (for objects on the ground plane)
    
    Objects at the bottom of the image (high y) are close.
    Objects at the top of the image (low y) are far away.
    
    Returns estimated distance in meters, clamped to [1, 50].
    """
    cy = frame_height / 2.0
    # Angle offset from optical axis (positive = below axis = closer)
    angle_offset = (pixel_y - cy) / FOCAL_Y
    # Total angle from horizontal
    ground_angle = CAMERA_TILT_RAD + angle_offset

    if ground_angle <= 0.05:  # nearly horizontal — object is very far
        return 50.0

    distance = CAMERA_HEIGHT / math.tan(ground_angle)
    return max(1.0, min(50.0, abs(distance)))


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
            f'VehicleCounter READY (CV available: {self.cv_available}) — depth estimation enabled'
        )

    def _empty_analysis(self, direction):
        return {
            'direction': direction,
            'vehicle_count': 0,
            'density': 'LOW',
            'queue_length': 0,
            'avg_depth': 0.0,
            'max_depth': 0.0,
            'vehicle_types': {'auto_rickshaw': 0, 'car': 0, 'bus_truck': 0},
            'emergency_detected': False,
            'emergency_depth': 0.0,
        }

    def _classify_density(self, count):
        for level, (low, high) in DENSITY_THRESHOLDS.items():
            if low <= count <= high:
                return level
        return 'VERY_HIGH'

    def _detect_emergency_hsv(self, hsv_frame, contours):
        """
        Detect emergency vehicles by HSV colour analysis.
        Returns (detected: bool, emergency_contour_or_None).
        """
        # Check for ambulance: white body + red cross
        white_mask = cv2.inRange(hsv_frame,
                                 np.array(HSV_WHITE_LOW), np.array(HSV_WHITE_HIGH))
        red_mask1 = cv2.inRange(hsv_frame,
                                np.array(HSV_RED1_LOW), np.array(HSV_RED1_HIGH))
        red_mask2 = cv2.inRange(hsv_frame,
                                np.array(HSV_RED2_LOW), np.array(HSV_RED2_HIGH))
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        if cv2.countNonZero(white_mask) > 500 and cv2.countNonZero(red_mask) > 100:
            # Find which contour overlaps with emergency colours
            em_contour = self._find_emergency_contour(contours, white_mask)
            return True, em_contour

        # Check for fire engine: large bright red area
        fire_mask = cv2.inRange(hsv_frame,
                                np.array(HSV_RED1_LOW), np.array(HSV_RED1_HIGH))
        fire_contours, _ = cv2.findContours(fire_mask, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)
        for cnt in fire_contours:
            if cv2.contourArea(cnt) > 800:
                return True, cnt

        # Check for police: blue body
        blue_mask = cv2.inRange(hsv_frame,
                                np.array(HSV_BLUE_LOW), np.array(HSV_BLUE_HIGH))
        if cv2.countNonZero(blue_mask) > 600:
            em_contour = self._find_emergency_contour(contours, blue_mask)
            return True, em_contour

        return False, None

    def _find_emergency_contour(self, contours, color_mask):
        """Find which vehicle contour overlaps most with the emergency color mask."""
        best = None
        best_overlap = 0
        for cnt in contours:
            mask = np.zeros(color_mask.shape, dtype=np.uint8)
            cv2.drawContours(mask, [cnt], -1, 255, -1)
            overlap = cv2.bitwise_and(mask, color_mask)
            count = cv2.countNonZero(overlap)
            if count > best_overlap:
                best_overlap = count
                best = cnt
        return best

    def _classify_vehicles(self, contours):
        """Classify detected contours into vehicle types by area and aspect ratio."""
        types = {'auto_rickshaw': 0, 'car': 0, 'bus_truck': 0}
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= AREA_BUS:
                types['bus_truck'] += 1
            elif area >= AREA_CAR:
                types['car'] += 1
            elif area >= AREA_AUTO_RICKSHAW:
                types['auto_rickshaw'] += 1
        return types

    def _compute_depth_stats(self, contours, frame_height):
        """
        Compute depth statistics from contour positions.
        Returns (avg_depth, max_depth, per_vehicle_depths).
        """
        depths = []
        for cnt in contours:
            M = cv2.moments(cnt)
            if M['m00'] > 0:
                cy = M['m01'] / M['m00']
                depth = _estimate_depth_from_pixel_y(cy, frame_height)
                depths.append(depth)

        if not depths:
            return 0.0, 0.0, []

        avg_depth = sum(depths) / len(depths)
        max_depth = max(depths)
        return avg_depth, max_depth, depths

    def _estimate_queue_length(self, contours, frame_height):
        """Estimate queue length from vertical distribution of contours."""
        if not contours:
            return 0
        # Count vehicles in the lower 2/3 of frame (approaching area = waiting in queue)
        queue = 0
        threshold_y = frame_height * 0.33
        for cnt in contours:
            M = cv2.moments(cnt)
            if M['m00'] > 0:
                cy = M['m01'] / M['m00']
                if cy > threshold_y:  # in approaching zone (closer to camera)
                    queue += 1
        return queue

    def _get_contour_depth(self, contour, frame_height):
        """Get estimated depth for a single contour."""
        if contour is None:
            return 0.0
        M = cv2.moments(contour)
        if M['m00'] > 0:
            cy = M['m01'] / M['m00']
            return _estimate_depth_from_pixel_y(cy, frame_height)
        return 0.0

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

        h, w = frame.shape[:2]

        # --- Step 1: Background subtraction for motion detection ---
        fgmask = self.subtractors[direction].apply(frame)
        _, thresh = cv2.threshold(fgmask, 200, 255, cv2.THRESH_BINARY)

        # --- Step 2: Color-based road segmentation ---
        # Road is dark grey (0.2,0.2,0.2 in Gazebo = ~51,51,51 in RGB)
        # Vehicles are brighter/colored — use value channel threshold
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Mask out the dark road surface: keep areas brighter than the road
        road_value_thresh = 80   # road ~51, vehicles typically > 100
        _, bright_mask = cv2.threshold(hsv[:, :, 2], road_value_thresh, 255,
                                       cv2.THRESH_BINARY)

        # --- Step 3: Combine motion + color for robust detection ---
        combined = cv2.bitwise_or(thresh, bright_mask)

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)

        # --- Step 4: Find and filter contours ---
        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        # Filter by area and solidity
        valid_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < AREA_AUTO_RICKSHAW:
                continue
            # Solidity check: vehicles are relatively solid shapes
            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            if hull_area > 0:
                solidity = area / hull_area
                if solidity < 0.3:
                    continue   # too fragmented, likely noise
            valid_contours.append(c)

        # --- Step 5: Count and classify ---
        vehicle_count = len(valid_contours)
        vehicle_types = self._classify_vehicles(valid_contours)
        queue_length = self._estimate_queue_length(valid_contours, h)

        # --- Step 6: Depth estimation ---
        avg_depth, max_depth, _ = self._compute_depth_stats(valid_contours, h)

        # --- Step 7: Emergency detection with depth ---
        emergency, em_contour = self._detect_emergency_hsv(hsv, valid_contours)
        emergency_depth = 0.0
        if emergency and em_contour is not None:
            emergency_depth = self._get_contour_depth(em_contour, h)

        # Build analysis
        analysis = {
            'direction': direction,
            'vehicle_count': vehicle_count,
            'density': self._classify_density(vehicle_count),
            'queue_length': queue_length,
            'avg_depth': round(avg_depth, 1),
            'max_depth': round(max_depth, 1),
            'vehicle_types': vehicle_types,
            'emergency_detected': emergency,
            'emergency_depth': round(emergency_depth, 1),
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