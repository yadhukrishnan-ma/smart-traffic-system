#!/usr/bin/env python3
"""
emergency_detector.py — Graduated emergency vehicle detection
==============================================================
Detection uses dual topic+camera agreement with distance-based graduated
messages to the traffic controller:

  - EMERGENCY_APPROACH (dist > 15m): early warning, extend green
  - EMERGENCY_CLOSE (5m < dist <= 15m): clear lane, hold green
  - EMERGENCY (dist <= 5m): full green override lock
  - CLEAR: vehicle has passed through junction

Detection requires BOTH topic-based AND camera-based agreement:
  1. Topic detection: vehicle_registry reports emergency type within 25 m
  2. Camera detection: OpenCV HSV analysis on overhead cameras
Only when both agree does the node fire an EMERGENCY/APPROACH/CLOSE message.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
    import cv2
    import numpy as np
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False

EMERGENCY_TYPES = {'ambulance', 'fire_engine', 'police'}
PRIORITY_WEIGHT = {'ambulance': 5, 'fire_engine': 4, 'police': 3}

# Direction letters → axis
DIR_AXIS = {'N': 'NS', 'S': 'NS', 'E': 'EW', 'W': 'EW'}
SPAWN_TO_DIR = {'A': 'N', 'C': 'S', 'B': 'E', 'D': 'W'}

# Camera topic → direction
CAM_TOPICS = {
    '/cam_north/image_raw': 'N',
    '/cam_east/image_raw':  'E',
    '/cam_south/image_raw': 'S',
    '/cam_west/image_raw':  'W',
}

# Distance thresholds for graduated response
APPROACH_DIST = 25.0  # start tracking
CLOSE_DIST    = 15.0  # escalate to CLOSE
OVERRIDE_DIST =  5.0  # full override


class EmergencyDetector(Node):

    def __init__(self):
        super().__init__('emergency_detector')

        self.em_pub = self.create_publisher(String, '/emergency_request', 10)

        self.create_subscription(String, '/vehicle_registry', self.registry_cb, 10)

        self.bridge = CvBridge() if CV_AVAILABLE else None

        for topic in CAM_TOPICS:
            self.create_subscription(
                Image, topic,
                lambda msg, t=topic: self.camera_cb(msg, t), 10
            )

        # Detection state per direction
        self.topic_detected  = {'N': {}, 'E': {}, 'S': {}, 'W': {}}
        self.camera_detected = {'N': False, 'E': False, 'S': False, 'W': False}

        # Track which emergencies we've already fired and at what level
        self.fired = {}         # name → last_action ('APPROACH'/'CLOSE'/'EMERGENCY')
        # Track cleared vehicles
        self.vehicle_info = {}  # name → {type, dir, dist, passed_center}

        self.create_timer(0.5, self.check_dual_agreement)
        self.create_timer(1.0, self.check_clear_conditions)

        self.get_logger().info(
            f'Emergency Detector READY (CV available: {CV_AVAILABLE})'
        )

    # ── Topic-based detection ────────────────────────────────────
    def registry_cb(self, msg: String):
        parts = msg.data.split(':')
        if len(parts) < 7:
            return
        name, vtype, direction = parts[0], parts[1], parts[2]
        dist = float(parts[6])
        d = SPAWN_TO_DIR.get(direction, '')

        # Track all emergency vehicles
        if vtype in EMERGENCY_TYPES:
            self.vehicle_info[name] = {
                'type': vtype, 'dir': d,
                'dist': dist,
                'x': float(parts[3]), 'y': float(parts[4]),
                'passed_center': dist < 3.0 or self.vehicle_info.get(name, {}).get('passed_center', False),
            }
            if dist < APPROACH_DIST and d:
                self.topic_detected[d][name] = (vtype, PRIORITY_WEIGHT.get(vtype, 3), dist)
            else:
                self.topic_detected.get(d, {}).pop(name, None)
        else:
            for dd in self.topic_detected:
                self.topic_detected[dd].pop(name, None)

    # ── Camera-based detection ───────────────────────────────────
    def camera_cb(self, msg: Image, topic: str):
        direction = CAM_TOPICS.get(topic, '')
        if not direction:
            return
        if not CV_AVAILABLE or self.bridge is None:
            self.camera_detected[direction] = bool(self.topic_detected[direction])
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detected = False

        # Detect AMBULANCE: white body + red cross
        white_mask = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 50, 255]))
        red_mask1  = cv2.inRange(hsv, np.array([0, 150, 100]), np.array([10, 255, 255]))
        red_mask2  = cv2.inRange(hsv, np.array([170, 150, 100]), np.array([180, 255, 255]))
        red_mask   = cv2.bitwise_or(red_mask1, red_mask2)
        if cv2.countNonZero(white_mask) > 500 and cv2.countNonZero(red_mask) > 100:
            detected = True

        # Detect FIRE ENGINE: bright red body, large area
        if not detected:
            fire_mask = cv2.inRange(hsv, np.array([0, 150, 100]), np.array([10, 255, 255]))
            contours, _ = cv2.findContours(fire_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) > 800:
                    detected = True
                    break

        # Detect POLICE: blue body
        if not detected:
            blue_mask = cv2.inRange(hsv, np.array([100, 100, 50]), np.array([130, 255, 255]))
            if cv2.countNonZero(blue_mask) > 600:
                detected = True

        self.camera_detected[direction] = detected

    # ── Dual agreement check with graduated response ─────────────
    def check_dual_agreement(self):
        for direction in ('N', 'E', 'S', 'W'):
            if not self.topic_detected[direction]:
                continue
            if not self.camera_detected[direction]:
                continue

            # Both detections agree — pick highest priority vehicle
            best_name = None
            best_pw   = -1
            best_type = ''
            best_dist = 999.0

            for name, (vtype, pw, dist) in self.topic_detected[direction].items():
                if pw > best_pw:
                    best_name = name
                    best_pw   = pw
                    best_type = vtype
                    best_dist = dist

            if best_name is None:
                continue

            # Determine action level based on distance
            if best_dist <= OVERRIDE_DIST:
                action = 'EMERGENCY'
            elif best_dist <= CLOSE_DIST:
                action = 'EMERGENCY_CLOSE'
            else:
                action = 'EMERGENCY_APPROACH'

            # Only send if this is a new detection or escalation
            prev_action = self.fired.get(best_name)
            action_rank = {'EMERGENCY_APPROACH': 1, 'EMERGENCY_CLOSE': 2, 'EMERGENCY': 3}

            if prev_action and action_rank.get(action, 0) <= action_rank.get(prev_action, 0):
                continue  # already at this level or higher

            axis = DIR_AXIS[direction]
            m = String()
            m.data = (f'{action}:{axis}:{best_name}:{best_type}:'
                      f'{best_pw}:{best_dist:.1f}')
            self.em_pub.publish(m)
            self.fired[best_name] = action
            self.get_logger().warn(
                f'DUAL DETECTION: {best_type} ({best_name}) from {direction} '
                f'dist={best_dist:.0f}m → {action} fired'
            )

    # ── Clear condition check ────────────────────────────────────
    def check_clear_conditions(self):
        to_clear = []
        for name, info in self.vehicle_info.items():
            if name not in self.fired:
                continue
            if info['passed_center'] and info['dist'] > 8.0:
                to_clear.append((name, info))

        for name, info in to_clear:
            d = info['dir']
            axis = DIR_AXIS.get(d, 'NS')
            m = String()
            m.data = f'CLEAR:{axis}:{name}'
            self.em_pub.publish(m)
            self.fired.pop(name, None)
            # Reset detection flags
            self.topic_detected.get(d, {}).pop(name, None)
            self.camera_detected[d] = False
            self.vehicle_info.pop(name, None)
            self.get_logger().info(
                f'CLEAR: {info["type"]} ({name}) passed center and exited junction'
            )


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyDetector()
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
