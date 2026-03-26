#!/usr/bin/env python3
"""
phase1_controller.py — GREEN LIGHT FIX
=======================================
Root cause of green-light-but-no-move bug:
  _follow_factor() used pure distance — no direction check.
  Car 1 (front of queue) saw Car 2 (behind it, 4m) → factor=0 → blocked itself.

Fix:
  Forward-cone check using dot product.
  Only vehicles within ±60° of forward heading count.
  Vehicles behind you are completely ignored.
  Car 1 at front of queue has nothing ahead → factor=1.0 → moves on green. ✓
  Car 2 sees Car 1 ahead → factor=0 → waits. ✓ (correct following behaviour)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates

# Path prefix → traffic signal direction
PATH_TO_DIR = {'A': 'N', 'B': 'E', 'C': 'S', 'D': 'W'}

PATHS = {
    # ── From A (North, heading south) ──
    'A_to_C': [
        ( 2.0,  40.0),
        ( 2.0,  25.0),
        ( 2.0,  14.0),
        ( 2.0,   7.5),
        ( 2.0,   3.0),
        ( 2.0,   0.0),
        ( 2.0,  -3.0),
        ( 2.0,  -7.5),
        ( 2.0, -25.0),
        ( 2.0, -44.0),
        ( 2.0, -55.0),
    ],
    'A_to_B': [
        (-2.0,  40.0),
        (-2.0,  25.0),
        (-2.0,  14.0),
        (-2.0,   7.5),
        (-2.0,   4.0),
        (-2.0,   2.0),
        (-1.0,   0.5),
        ( 0.0,  -0.5),
        ( 1.0,  -1.5),
        ( 2.5,  -2.0),
        ( 6.0,  -2.0),
        (15.0,  -2.0),
        (30.0,  -2.0),
        (44.0,  -2.0),
    ],
    'A_to_D': [
        (-2.0,  40.0),
        (-2.0,  25.0),
        (-2.0,  14.0),
        (-2.0,   7.5),
        (-2.0,   4.0),
        (-2.5,   2.5),
        (-4.0,   2.0),
        (-6.0,   2.0),
        (-15.0,  2.0),
        (-30.0,  2.0),
        (-44.0,  2.0),
    ],
    # ── From B (East, heading west) ──
    'B_to_D': [
        ( 40.0, -2.0),
        ( 25.0, -2.0),
        ( 14.0, -2.0),
        (  7.5, -2.0),
        (  3.0, -2.0),
        (  0.0, -2.0),
        ( -3.0, -2.0),
        ( -7.5, -2.0),
        (-25.0, -2.0),
        (-44.0, -2.0),
        (-55.0, -2.0),
    ],
    'B_to_A': [
        ( 40.0,  2.0),
        ( 25.0,  2.0),
        ( 14.0,  2.0),
        (  7.5,  2.0),
        (  4.0,  2.0),
        (  2.0,  2.0),
        (  0.5,  1.0),
        (  0.0,  0.0),
        ( -0.5,  1.0),
        ( -2.0,  2.5),
        ( -2.0,  6.0),
        ( -2.0, 15.0),
        ( -2.0, 30.0),
        ( -2.0, 44.0),
    ],
    'B_to_C': [
        ( 40.0,  2.0),
        ( 25.0,  2.0),
        ( 14.0,  2.0),
        (  7.5,  2.0),
        (  4.0,  2.0),
        (  2.5,  2.5),
        (  2.0,  4.0),
        (  2.0,  6.0),
        (  2.0, 15.0),
        (  2.0, 30.0),
        (  2.0, 44.0),
    ],
    # ── From C (South, heading north) ──
    'C_to_A': [
        (-2.0, -40.0),
        (-2.0, -25.0),
        (-2.0, -14.0),
        (-2.0,  -7.5),
        (-2.0,  -3.0),
        (-2.0,   0.0),
        (-2.0,   3.0),
        (-2.0,   7.5),
        (-2.0,  25.0),
        (-2.0,  44.0),
        (-2.0,  55.0),
    ],
    'C_to_B': [
        ( 2.0, -40.0),
        ( 2.0, -25.0),
        ( 2.0, -14.0),
        ( 2.0,  -7.5),
        ( 2.0,  -4.0),
        ( 2.0,  -2.0),
        ( 1.0,  -0.5),
        ( 0.0,   0.0),
        ( 1.0,   0.5),
        ( 2.5,  -2.0),
        ( 6.0,  -2.0),
        (15.0,  -2.0),
        (30.0,  -2.0),
        (44.0,  -2.0),
    ],
    'C_to_D': [
        ( 2.0, -40.0),
        ( 2.0, -25.0),
        ( 2.0, -14.0),
        ( 2.0,  -7.5),
        ( 2.0,  -4.0),
        ( 2.5,  -2.5),
        ( 4.0,  -2.0),
        ( 6.0,  -2.0),
        (15.0,  -2.0),
        (30.0,  -2.0),
        (44.0,  -2.0),
    ],
    # ── From D (West, heading east) ──
    'D_to_B': [
        (-40.0,  2.0),
        (-25.0,  2.0),
        (-14.0,  2.0),
        ( -7.5,  2.0),
        ( -3.0,  2.0),
        (  0.0,  2.0),
        (  3.0,  2.0),
        (  7.5,  2.0),
        ( 25.0,  2.0),
        ( 44.0,  2.0),
        ( 55.0,  2.0),
    ],
    'D_to_A': [
        (-40.0, -2.0),
        (-25.0, -2.0),
        (-14.0, -2.0),
        ( -7.5, -2.0),
        ( -4.0, -2.0),
        ( -2.0, -2.0),
        ( -0.5, -1.0),
        (  0.5,  0.0),
        (  1.5,  1.0),
        (  2.0,  2.5),
        (  2.0,  6.0),
        (  2.0, 15.0),
        (  2.0, 30.0),
        (  2.0, 44.0),
    ],
    'D_to_C': [
        (-40.0, -2.0),
        (-25.0, -2.0),
        (-14.0, -2.0),
        ( -7.5, -2.0),
        ( -4.0, -2.0),
        ( -2.5, -2.5),
        ( -2.0, -4.0),
        ( -2.0, -6.0),
        ( -2.0,-15.0),
        ( -2.0,-30.0),
        ( -2.0,-44.0),
    ],
}

STRAIGHT_PATHS = {'A_to_C', 'B_to_D', 'C_to_A', 'D_to_B'}

CRUISE_SPEED   = 2.5
APPROACH_SPEED = 1.8
CAUTION_SPEED  = 1.2
TURN_SPEED     = 0.8
PRETURN_SPEED  = 1.0

K_STEER     = 0.55
MAX_ANGULAR = 0.7
STEER_ALPHA = 0.7

WP_THRESHOLD_STRAIGHT = 2.5
WP_THRESHOLD_TURN     = 0.9

# Following distance — applies only to vehicles AHEAD (forward cone)
VEHICLE_LENGTHS = {
    'bike': 1.5,
    'car': 2.2,
    'police': 2.4,
    'ambulance': 2.8,
    'bus': 4.5,
    'fire_engine': 4.5,
    'truck': 5.0
}
FORWARD_CONE_RAD = math.radians(70)   # ±70° forward cone


def _angle_at_b(ax, ay, bx, by, cx, cy):
    v1x, v1y = ax - bx, ay - by
    v2x, v2y = cx - bx, cy - by
    dot = v1x * v2x + v1y * v2y
    mag = math.hypot(v1x, v1y) * math.hypot(v2x, v2y)
    if mag < 1e-6:
        return math.pi
    return math.acos(max(-1.0, min(1.0, dot / mag)))


class Phase1Controller(Node):
    def __init__(self):
        super().__init__('vehicle_controller')

        self.declare_parameter('vehicle_name', 'vehicle_0')
        self.declare_parameter('vehicle_type', 'car')
        self.declare_parameter('path', 'A_to_C')

        self.name    = str(self.get_parameter('vehicle_name').value)
        self.vtype   = str(self.get_parameter('vehicle_type').value)
        self.path_id = str(self.get_parameter('path').value)

        self.is_turning   = self.path_id not in STRAIGHT_PATHS
        self.wp_threshold = (WP_THRESHOLD_TURN if self.is_turning
                             else WP_THRESHOLD_STRAIGHT)
        self.waypoints = PATHS.get(self.path_id, PATHS['A_to_C'])
        self.wp_index  = 0
        self._preturn_indices = self._find_preturn_indices()

        self.world_x = self.world_y = self.world_yaw = 0.0
        self.pos_received   = False
        self._prev_steer    = 0.0
        # Derive phase_dir from path prefix (A→N, B→E, C→S, D→W)
        path_prefix = self.path_id[0] if self.path_id else 'A'
        self.phase_dir      = PATH_TO_DIR.get(path_prefix, 'N')
        self.traffic_phase  = 'ALL_RED'
        self.nearby_vehicles = []

        self.passed_center   = False
        self.finished        = False
        self._signal_stopped = False
        self._follow_stopped = False

        self.cmd_pub  = self.create_publisher(Twist,  'cmd_vel',          10)
        self.reg_pub  = self.create_publisher(String, '/vehicle_registry', 10)
        self.done_pub = self.create_publisher(String, '/vehicle_done',     10)

        self.create_subscription(ModelStates, '/gazebo/model_states',  self.model_states_cb, 10)
        self.create_subscription(String,      '/traffic_phase',        self.traffic_cb,      10)
        self.create_subscription(String,      '/vehicle_positions',    self.positions_cb,    10)

        self.create_timer(0.1, self.control_loop)
        self.create_timer(1.0, self.publish_registry)

        self.get_logger().info(
            f'[{self.name}] path={self.path_id} turning={self.is_turning}'
        )

    def _find_preturn_indices(self):
        wps = self.waypoints
        preturn = set()
        for i in range(1, len(wps) - 1):
            angle = _angle_at_b(
                wps[i-1][0], wps[i-1][1],
                wps[i][0],   wps[i][1],
                wps[i+1][0], wps[i+1][1]
            )
            if angle < math.radians(140):
                for offset in range(0, 4):
                    idx = i - offset
                    if idx >= 0:
                        preturn.add(idx)
        return preturn

    def model_states_cb(self, msg: ModelStates):
        try:
            idx = list(msg.name).index(self.name)
        except ValueError:
            return
        pose = msg.pose[idx]
        self.world_x = pose.position.x
        self.world_y = pose.position.y
        q = pose.orientation
        self.world_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pos_received = True

    def traffic_cb(self, msg: String):
        self.traffic_phase = msg.data

    def positions_cb(self, msg: String):
        self.nearby_vehicles = []
        if not msg.data:
            return
        for entry in msg.data.split(','):
            parts = entry.split(':')
            if len(parts) >= 4 and parts[0] != self.name:
                try:
                    self.nearby_vehicles.append(
                        (float(parts[1]), float(parts[2]), parts[3])
                    )
                except ValueError:
                    pass

    def publish_registry(self):
        d = math.hypot(self.world_x, self.world_y)
        # Use path prefix as spawn direction letter
        spawn_dir = self.path_id[0] if self.path_id else 'A'
        speed = abs(getattr(self, '_last_speed', 0.0))
        m = String()
        m.data = (f'{self.name}:{self.vtype}:{spawn_dir}:'
                  f'{self.world_x:.2f}:{self.world_y:.2f}:'
                  f'{speed:.2f}:{d:.2f}:{self.path_id}')
        self.reg_pub.publish(m)

    @staticmethod
    def _norm(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def _follow_factor(self):
        """
        Forward-cone follow check — only vehicles AHEAD count.

        How it works:
          1. Compute vector from this car to other car: (dx, dy)
          2. Compute dot product with forward direction (cos(yaw), sin(yaw))
          3. If dot product > 0 AND angle < FORWARD_CONE_RAD: vehicle is ahead
          4. Apply distance-based speed factor only for vehicles ahead

        This means:
          - Car 1 (front of queue): nothing ahead → factor=1.0 → moves on green ✓
          - Car 2 (behind Car 1): Car 1 is ahead → factor=0 if within safe_dist ✓
          - Car 1: Car 2 is BEHIND → completely ignored → can move freely ✓
        """
        factor = 1.0
        fwd_x = math.cos(self.world_yaw)   # forward unit vector X
        fwd_y = math.sin(self.world_yaw)   # forward unit vector Y
        my_len = VEHICLE_LENGTHS.get(self.vtype, 2.2)

        for vx, vy, otype in self.nearby_vehicles:
            dx = vx - self.world_x
            dy = vy - self.world_y
            dist = math.hypot(dx, dy)

            if dist < 0.5:
                continue   # same position ghost

            # Dot product: positive = vehicle is in front half-space
            dot = (dx * fwd_x + dy * fwd_y) / dist
            # dot = cos(angle between forward and direction-to-other-vehicle)
            # dot > cos(70°) = 0.342 means angle < 70° = vehicle is ahead

            if dot < math.cos(FORWARD_CONE_RAD):
                continue   # vehicle is behind or to the side — IGNORE

            other_len = VEHICLE_LENGTHS.get(otype, 2.2)
            safe_dist = (my_len / 2.0) + (other_len / 2.0) + 1.5  # 1.5m static gap
            slow_dist = safe_dist + 3.0

            # Vehicle is ahead in the cone
            if dist < safe_dist:
                factor = 0.0
                if not self._follow_stopped:
                    self._follow_stopped = True
                    self.get_logger().info(
                        f'[{self.name}] FOLLOW STOP — {otype} {dist:.1f}m ahead (req {safe_dist:.1f}m)'
                    )
                return factor
            elif dist < slow_dist:
                factor = min(factor, 0.5)

        if factor > 0.0 and self._follow_stopped:
            self._follow_stopped = False
            self.get_logger().info(f'[{self.name}] Follow clear')
        return factor

    def _should_stop_at_signal(self, dist_ctr):
        if self.passed_center:
            return False
            
        my_len = VEHICLE_LENGTHS.get(self.vtype, 2.2) / 2.0
        target_y    = 5.0 + my_len
        target_dist = math.hypot(2.0, target_y)

        # Commit to intersection crossing if we are past the stop line
        if dist_ctr < target_dist - 0.5:
            self.committed_to_intersection = True
            
        if getattr(self, 'committed_to_intersection', False):
            return False

        # Stop monitoring signal if we are far away
        if dist_ctr > target_dist + 0.5 or dist_ctr < 2.0:
            return False

        phase = self.traffic_phase
        if phase == 'ALL_RED':
            should_stop = True
        elif phase.startswith('PHASE:'):
            parts = phase.split(':')
            if len(parts) >= 3:
                phase_dir, phase_state = parts[1], parts[2]
                if self.phase_dir in phase_dir:
                    should_stop = (phase_state != 'GREEN' and
                                   not (phase_state == 'YELLOW' and dist_ctr <= 4.0))
                else:
                    should_stop = True
            else:
                should_stop = True
        else:
            should_stop = True

        if should_stop and not self._signal_stopped:
            self._signal_stopped = True
            self.get_logger().info(
                f'[{self.name}] RED STOP {dist_ctr:.1f}m — {phase}'
            )
        elif not should_stop and self._signal_stopped:
            self._signal_stopped = False
            self.get_logger().info(
                f'[{self.name}] GREEN GO — {phase}'
            )
        return should_stop

    def _zone_speed(self, dist_ctr):
        if self.passed_center:
            return CRUISE_SPEED
        if self.wp_index in self._preturn_indices:
            return PRETURN_SPEED
        if dist_ctr > 12.0:
            return CRUISE_SPEED
        if dist_ctr > 7.0:
            return APPROACH_SPEED
        if self.is_turning:
            return TURN_SPEED
        return CAUTION_SPEED

    def control_loop(self):
        if not self.pos_received or self.finished:
            return

        x, y, yaw = self.world_x, self.world_y, self.world_yaw
        dist_ctr = math.hypot(x, y)
        tw = Twist()

        if dist_ctr < 3.0:
            self.passed_center = True

        if self.wp_index >= len(self.waypoints):
            self._finish(tw)
            return

        tx, ty = self.waypoints[self.wp_index]
        dx, dy = tx - x, ty - y
        dist_wp = math.hypot(dx, dy)

        # Check if waypoint is behind the vehicle (overshot)
        fwd_x = math.cos(yaw)
        fwd_y = math.sin(yaw)
        dot_wp = dx * fwd_x + dy * fwd_y

        if dist_wp < self.wp_threshold or dot_wp < 0:
            self.wp_index += 1
            if self.wp_index >= len(self.waypoints):
                self._finish(tw)
                return
            tx, ty = self.waypoints[self.wp_index]
            dx, dy = tx - x, ty - y

        err = self._norm(math.atan2(dy, dx) - yaw)
        raw_steer = max(-MAX_ANGULAR, min(MAX_ANGULAR, K_STEER * err))
        smooth_steer = ((1.0 - STEER_ALPHA) * raw_steer
                        + STEER_ALPHA * self._prev_steer)
        self._prev_steer = smooth_steer

        speed = self._zone_speed(dist_ctr)

        # Signal has priority over following distance
        if self._should_stop_at_signal(dist_ctr):
            speed = 0.0
        else:
            # Only check following distance if signal allows movement
            speed *= self._follow_factor()

        if self.is_turning and dist_ctr < 8.0 and abs(err) > 0.5:
            speed *= 0.6

        self._last_speed = speed
        tw.linear.x  = float(speed)
        tw.angular.z = float(smooth_steer) if speed > 0.05 else 0.0
        self.cmd_pub.publish(tw)

    def _finish(self, tw):
        tw.linear.x = tw.angular.z = 0.0
        self.cmd_pub.publish(tw)
        if not self.finished:
            self.finished = True
            m = String()
            m.data = self.name
            self.done_pub.publish(m)
            self.get_logger().info(f'[{self.name}] DONE — {self.path_id}')


def main(args=None):
    rclpy.init(args=args)
    node = Phase1Controller()
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
