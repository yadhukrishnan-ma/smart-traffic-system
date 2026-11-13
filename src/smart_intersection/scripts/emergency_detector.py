#!/usr/bin/env python3
"""
emergency_detector.py — Ambulance-only graduated emergency detection
=====================================================================
Detects AMBULANCE only (no fire_engine, no police).
Uses dual topic + camera agreement:

  - EMERGENCY_APPROACH (dist > 15 m): early warning, extend green
  - EMERGENCY_CLOSE (5 m < dist ≤ 15 m): clear lane, hold green
  - EMERGENCY (dist ≤ 5 m): full green override lock
  - CLEAR: ambulance has passed through junction

Detection requires BOTH:
  1. Topic detection: vehicle_registry reports ambulance within 25 m
  2. Camera detection: OpenCV HSV analysis (white body + red cross)
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

EMERGENCY_TYPE  = 'ambulance'
PRIORITY_WEIGHT = {'ambulance': 5}

# Direction letters → axis
DIR_AXIS    = {'N': 'NS', 'S': 'NS', 'E': 'EW', 'W': 'EW'}
SPAWN_TO_DIR = {'A': 'N', 'C': 'S', 'B': 'E', 'D': 'W'}

# Camera topic → direction
CAM_TOPICS = {
    '/cam_north/image_raw': 'N',
    '/cam_east/image_raw':  'E',
    '/cam_south/image_raw': 'S',
    '/cam_west/image_raw':  'W',
}

# Distance thresholds
APPROACH_DIST = 50.0  # detect from spawn point (~44m from junction)
CLOSE_DIST    = 15.0  # escalate
OVERRIDE_DIST =  5.0  # full override


class EmergencyDetector(Node):

    def __init__(self):
        super().__init__('emergency_detector')

        self.em_pub = self.create_publisher(String, '/emergency_request', 10)
        self.create_subscription(String, '/vehicle_registry', self.registry_cb, 10)
        self.create_subscription(String, '/vehicle_done', self.done_cb, 10)

        self.bridge = CvBridge() if CV_AVAILABLE else None

        for topic in CAM_TOPICS:
            self.create_subscription(
                Image, topic,
                lambda msg, t=topic: self.camera_cb(msg, t), 10
            )

        # Detection state per direction
        self.topic_detected  = {'N': {}, 'E': {}, 'S': {}, 'W': {}}
        self.camera_detected = {'N': False, 'E': False, 'S': False, 'W': False}

        # Track which emergencies we've fired and at what level
        self.fired        = {}   # name → last_action
        self.vehicle_info = {}   # name → {type, dir, dist, passed_center}

        self.create_timer(0.5, self.check_dual_agreement)
        self.create_timer(1.0, self.check_clear_conditions)

        self.get_logger().info(
            f'Emergency Detector READY — ambulance only (CV: {CV_AVAILABLE})'
        )

    # ── Topic-based detection ─────────────────────────────────────────────
    def registry_cb(self, msg: String):
        parts = msg.data.split(':')
        if len(parts) < 7:
            return
        name, vtype, direction = parts[0], parts[1], parts[2]
        dist = float(parts[6])
        d = SPAWN_TO_DIR.get(direction, '')

        if vtype == EMERGENCY_TYPE:
            self.vehicle_info[name] = {
                'type': vtype, 'dir': d, 'dist': dist,
                'x': float(parts[3]), 'y': float(parts[4]),
                'passed_center': (
                    dist < 3.0 or
                    self.vehicle_info.get(name, {}).get('passed_center', False)
                ),
            }
            if dist < APPROACH_DIST and d:
                self.topic_detected[d][name] = (vtype, PRIORITY_WEIGHT.get(vtype, 5), dist)
            else:
                self.topic_detected.get(d, {}).pop(name, None)
        else:
            # Non-ambulance vehicles: clear from all detections
            for dd in self.topic_detected:
                self.topic_detected[dd].pop(name, None)

    def done_cb(self, msg: String):
        name = msg.data
        self.fired.pop(name, None)
        self.vehicle_info.pop(name, None)
        for d in self.topic_detected:
            self.topic_detected[d].pop(name, None)
        self.get_logger().info(f'Emergency Detector: cleared done vehicle {name}')

    # ── Camera-based detection — ambulance only ───────────────────────────
    def camera_cb(self, msg: Image, topic: str):
        direction = CAM_TOPICS.get(topic, '')
        if not direction:
            return

        if not CV_AVAILABLE or self.bridge is None:
            # Without OpenCV, fall back to topic-only detection
            self.camera_detected[direction] = bool(self.topic_detected[direction])
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detected = False

        # AMBULANCE: white body + red cross
        white_mask = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 50, 255]))
        red_mask1  = cv2.inRange(hsv, np.array([0, 150, 100]), np.array([10, 255, 255]))
        red_mask2  = cv2.inRange(hsv, np.array([170, 150, 100]), np.array([180, 255, 255]))
        red_mask   = cv2.bitwise_or(red_mask1, red_mask2)

        if (cv2.countNonZero(white_mask) > 500 and
                cv2.countNonZero(red_mask) > 100):
            detected = True

        self.camera_detected[direction] = detected

    # ── Dual agreement check ──────────────────────────────────
    def check_dual_agreement(self):
        for direction in ('N', 'E', 'S', 'W'):
            if not self.topic_detected[direction]:
                continue

            # Pick highest priority vehicle in this direction
            best_name, best_pw, best_type, best_dist = None, -1, '', 999.0
            for name, (vtype, pw, dist) in self.topic_detected[direction].items():
                if pw > best_pw:
                    best_name, best_pw, best_type, best_dist = name, pw, vtype, dist

            if best_name is None:
                continue

            # Camera confirmation policy:
            #  - CLOSE / OVERRIDE (dist ≤ 15 m): topic alone is sufficient
            #    (ambulance is physically close — false positives impossible)
            #  - APPROACH (dist > 15 m): require camera to avoid false positives
            camera_confirmed = self.camera_detected[direction]
            close_range = (best_dist <= CLOSE_DIST)

            if not close_range and not camera_confirmed:
                continue  # far away — still need camera

            # Determine action level
            if best_dist <= OVERRIDE_DIST:
                action = 'EMERGENCY'
            elif best_dist <= CLOSE_DIST:
                action = 'EMERGENCY_CLOSE'
            else:
                action = 'EMERGENCY_APPROACH'

            # Only escalate, never de-escalate
            prev_action = self.fired.get(best_name)
            action_rank = {'EMERGENCY_APPROACH': 1, 'EMERGENCY_CLOSE': 2, 'EMERGENCY': 3}
            if prev_action and action_rank.get(action, 0) <= action_rank.get(prev_action, 0):
                continue

            axis = DIR_AXIS[direction]
            m = String()
            m.data = f'{action}:{axis}:{best_name}:{best_type}:{best_pw}:{best_dist:.1f}'
            self.em_pub.publish(m)
            self.fired[best_name] = action
            src = 'topic+camera' if camera_confirmed else 'topic-only(close)'
            self.get_logger().warn(
                f'AMBULANCE [{src}] from {direction} '
                f'dist={best_dist:.0f}m → {action}'
            )

    # ── Clear condition check ─────────────────────────────────────────────
    def check_clear_conditions(self):
        to_clear = []
        for name, info in self.vehicle_info.items():
            if name not in self.fired:
                continue
            if info['passed_center'] and info['dist'] > 8.0:
                to_clear.append((name, info))

        for name, info in to_clear:
            d    = info['dir']
            axis = DIR_AXIS.get(d, 'NS')
            m    = String()
            m.data = f'CLEAR:{axis}:{name}'
            self.em_pub.publish(m)
            self.fired.pop(name, None)
            self.topic_detected.get(d, {}).pop(name, None)
            self.camera_detected[d] = False
            self.vehicle_info.pop(name, None)
            self.get_logger().info(
                f'CLEAR: ambulance ({name}) passed center and exited junction'
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
