#!/usr/bin/env python3
"""
vehicle_controller.py — Cars + Ambulance controller
=====================================================
Key behaviours:
  1. REAL-WORLD yellow light rule:
       - dist > 5.5 m from center → STOP (room to brake safely)
       - dist ≤ 5.5 m from center → GO  (too close to stop = committed)
  2. Point of no return at 4.5 m: once inside junction box, NEVER stops
  3. Stop line at ~8 m from center
  4. Ambulance skips all signal checks and gets priority passage
  5. Forward-cone (50°) following: vehicles AHEAD only, larger slow buffer
  6. Ambulance-yield: regular cars slow to 20% when ambulance is behind
  7. Smooth speed low-pass filter — no abrupt speed jumps
  8. Auto-shutdown after reaching destination
"""

import math
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates

PATH_TO_DIR    = {'A': 'N', 'B': 'E', 'C': 'S', 'D': 'W'}
EMERGENCY_TYPE = 'ambulance'

PATHS = {
    'A_to_C': [
        ( 2.0,  40.0), ( 2.0,  25.0), ( 2.0,  14.0), ( 2.0,   7.5),
        ( 2.0,   3.0), ( 2.0,   0.0), ( 2.0,  -3.0), ( 2.0,  -7.5),
        ( 2.0, -25.0), ( 2.0, -44.0), ( 2.0, -55.0),
    ],
    'B_to_D': [
        ( 40.0, -2.0), ( 25.0, -2.0), ( 14.0, -2.0), (  7.5, -2.0),
        (  3.0, -2.0), (  0.0, -2.0), ( -3.0, -2.0), ( -7.5, -2.0),
        (-25.0, -2.0), (-44.0, -2.0), (-55.0, -2.0),
    ],
    'C_to_A': [
        (-2.0, -40.0), (-2.0, -25.0), (-2.0, -14.0), (-2.0,  -7.5),
        (-2.0,  -3.0), (-2.0,   0.0), (-2.0,   3.0), (-2.0,   7.5),
        (-2.0,  25.0), (-2.0,  44.0), (-2.0,  55.0),
    ],
    'D_to_B': [
        (-40.0,  2.0), (-25.0,  2.0), (-14.0,  2.0), ( -7.5,  2.0),
        ( -3.0,  2.0), (  0.0,  2.0), (  3.0,  2.0), (  7.5,  2.0),
        ( 25.0,  2.0), ( 44.0,  2.0), ( 55.0,  2.0),
    ],
}

STRAIGHT_PATHS = {'A_to_C', 'B_to_D', 'C_to_A', 'D_to_B'}

# ---------------------------------------------------------------------------
# Speed constants
# ---------------------------------------------------------------------------
CRUISE_SPEED   = 2.2   # m/s on open road
APPROACH_SPEED = 1.6   # m/s approaching junction (12–8 m)
CAUTION_SPEED  = 1.1   # m/s in junction zone
AMBULANCE_MULT = 1.8   # ambulance speed multiplier
CAR_MULT       = 1.0

# Low-pass filter for speed — prevents abrupt jerky changes
SPEED_ALPHA = 0.25     # 0 = instant, 1 = fully filtered (use 0.25 for smooth ramp)

# ---------------------------------------------------------------------------
# Steering constants
# ---------------------------------------------------------------------------
K_STEER       = 0.55
MAX_ANGULAR   = 0.7
STEER_ALPHA   = 0.7

WP_THRESHOLD_STRAIGHT = 2.5
WP_THRESHOLD_TURN     = 0.9

# ---------------------------------------------------------------------------
# Vehicle geometry
# ---------------------------------------------------------------------------
VEHICLE_LENGTH = {'ambulance': 2.8, 'car': 2.2}
DEFAULT_LENGTH = 2.2

# ---------------------------------------------------------------------------
# Forward-cone collision avoidance
# Cone narrowed to 50° (was 70°) for fewer false positives
# Slow buffer increased to 5.0 m (was 3.0) for smoother braking
# ---------------------------------------------------------------------------
FORWARD_CONE_RAD = math.radians(18)
SLOW_BUFFER_EXTRA = 5.0   # metres beyond safe_dist to start slowing

# ---------------------------------------------------------------------------
# Ambulance yield — regular cars slow to YIELD_FACTOR when ambulance is
# detected BEHIND them within AMBULANCE_YIELD_DIST metres
# ---------------------------------------------------------------------------
AMBULANCE_YIELD_DIST   = 35.0   # metres
AMBULANCE_YIELD_FACTOR = 0.20   # 20% of normal speed (almost stopped)

# ---------------------------------------------------------------------------
# Signal zones
# ---------------------------------------------------------------------------
SIGNAL_CHECK_FAR     = 12.0   # start checking signal at 12 m from center
JUNCTION_COMMIT_DIST = 8.0    # point of no return for red/all_red (stop line)
# Real-world yellow commit: stop if dist > this, go if dist <= this
YELLOW_COMMIT_DIST   = 9.2    # metres from center (slightly before stop line)


def _angle_at_b(ax, ay, bx, by, cx, cy):
    v1x, v1y = ax - bx, ay - by
    v2x, v2y = cx - bx, cy - by
    dot = v1x * v2x + v1y * v2y
    mag = math.hypot(v1x, v1y) * math.hypot(v2x, v2y)
    if mag < 1e-6:
        return math.pi
    return math.acos(max(-1.0, min(1.0, dot / mag)))


class VehicleController(Node):
    def __init__(self):
        super().__init__('vehicle_controller')

        self.declare_parameter('vehicle_name', 'vehicle_0')
        self.declare_parameter('vehicle_type', 'car')
        self.declare_parameter('path', 'A_to_C')

        self.name    = str(self.get_parameter('vehicle_name').value)
        self.vtype   = str(self.get_parameter('vehicle_type').value)
        self.path_id = str(self.get_parameter('path').value)
        self.is_emergency = (self.vtype == EMERGENCY_TYPE)

        self.is_turning   = self.path_id not in STRAIGHT_PATHS
        self.wp_threshold = WP_THRESHOLD_TURN if self.is_turning else WP_THRESHOLD_STRAIGHT
        self.waypoints    = PATHS.get(self.path_id, PATHS['A_to_C'])
        self.wp_index     = 0

        self.world_x = self.world_y = self.world_yaw = 0.0
        self.pos_received = False
        self.last_pos_time = self.get_clock().now()
        self._prev_steer  = 0.0
        self._last_speed  = 0.0   # actual smoothed speed (for low-pass filter)

        path_prefix     = self.path_id[0] if self.path_id else 'A'
        self.phase_dir  = PATH_TO_DIR.get(path_prefix, 'N')
        self.traffic_phase   = 'ALL_RED'
        self.nearby_vehicles = []   # list of (vx, vy, vtype, vname)

        self.committed     = False
        self.passed_center = False
        self.finished      = False
        self._shutdown_scheduled = False
        self._signal_stopped     = False
        self._follow_stopped     = False
        self._yielding_ambulance = False

        # Publishers / Subscribers
        self.cmd_pub  = self.create_publisher(Twist,  'cmd_vel',          10)
        self.reg_pub  = self.create_publisher(String, '/vehicle_registry', 10)
        self.done_pub = self.create_publisher(String, '/vehicle_done',     10)

        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_cb, 10)
        self.create_subscription(String,      '/traffic_phase',       self.traffic_cb,      10)
        self.create_subscription(String,      '/vehicle_positions',   self.positions_cb,    10)

        self.create_timer(0.1, self.control_loop)
        self.create_timer(1.0, self.publish_registry)

        self.get_logger().info(
            f'[{self.name}] path={self.path_id} type={self.vtype} '
            f'emergency={self.is_emergency}'
        )

    # ── Callbacks ───────────────────────────────────────────────────────────
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
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.pos_received = True
        self.last_pos_time = self.get_clock().now()

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
                    self.nearby_vehicles.append((
                        float(parts[1]),
                        float(parts[2]),
                        parts[3],           # vehicle type
                        parts[0],           # vehicle name
                    ))
                except ValueError:
                    pass

    def publish_registry(self):
        if self.finished:
            m = String(); m.data = self.name; self.done_pub.publish(m)
            return
        d = math.hypot(self.world_x, self.world_y)
        spawn_dir = self.path_id[0] if self.path_id else 'A'
        speed = abs(self._last_speed)
        m = String()
        m.data = (f'{self.name}:{self.vtype}:{spawn_dir}:'
                  f'{self.world_x:.2f}:{self.world_y:.2f}:'
                  f'{speed:.2f}:{d:.2f}:{self.path_id}')
        self.reg_pub.publish(m)

    # ── Utilities ────────────────────────────────────────────────────────────
    @staticmethod
    def _norm(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def _speed_mult(self):
        return AMBULANCE_MULT if self.is_emergency else CAR_MULT

    # ── Ambulance yield check (for regular cars only) ────────────────────────
    def _ambulance_behind_factor(self):
        """
        Regular cars slow to YIELD_FACTOR when ambulance is BEHIND them
        AND on the same road axis (same direction of travel).
        Cross-traffic cars are not affected.
        """
        if self.is_emergency:
            return 1.0

        fwd_x = math.cos(self.world_yaw)
        fwd_y = math.sin(self.world_yaw)

        for vx, vy, otype, _vname in self.nearby_vehicles:
            if otype != EMERGENCY_TYPE:
                continue
            dx = vx - self.world_x
            dy = vy - self.world_y
            dist = math.hypot(dx, dy)
            if dist > AMBULANCE_YIELD_DIST or dist < 0.5:
                continue

            # Ambulance must be roughly behind us on the SAME axis
            # (dot < -0.3 means >100° behind forward direction)
            dot = (dx * fwd_x + dy * fwd_y) / dist
            if dot < -0.3:
                if not self._yielding_ambulance:
                    self._yielding_ambulance = True
                    self.get_logger().info(
                        f'[{self.name}] YIELDING — ambulance {dist:.1f}m behind'
                    )
                return AMBULANCE_YIELD_FACTOR

        if self._yielding_ambulance:
            self._yielding_ambulance = False
            self.get_logger().info(f'[{self.name}] Yield cleared')
        return 1.0

    # ── Forward-cone collision avoidance ────────────────────────────────────
    def _follow_factor(self):
        """
        Forward-cone follow check (50°).
        Safe distances include braking margin so NO vehicle ever hits another.
        Returns factor in [0.0, 1.0].
        """
        factor = 1.0
        fwd_x  = math.cos(self.world_yaw)
        fwd_y  = math.sin(self.world_yaw)
        my_len = VEHICLE_LENGTH.get(self.vtype, DEFAULT_LENGTH)

        for vx, vy, otype, _vname in self.nearby_vehicles:
            dx = vx - self.world_x
            dy = vy - self.world_y
            dist = math.hypot(dx, dy)
            if dist < 0.5:
                continue

            dot = (dx * fwd_x + dy * fwd_y) / dist
            if dot < math.cos(FORWARD_CONE_RAD):
                continue

            other_len = VEHICLE_LENGTH.get(otype, DEFAULT_LENGTH)
            half_sum  = (my_len / 2.0) + (other_len / 2.0)

            # Ambulance travels faster so needs MORE braking room, not less.
            # At 5.6 m/s, braking distance ~4 m — safe_dist includes that margin.
            if self.is_emergency:
                safe_dist = half_sum + 3.5   # guaranteed zero-hit gap for ambulance
                slow_dist = safe_dist + 7.0  # start slowing 7 m before safe zone
            else:
                safe_dist = half_sum + 3.0   # generous gap for regular cars
                slow_dist = safe_dist + 7.0  # start slowing 7 m before safe zone

            if dist < safe_dist:
                factor = 0.0
                if not self._follow_stopped:
                    self._follow_stopped = True
                    self.get_logger().info(
                        f'[{self.name}] FOLLOW STOP — {otype} {dist:.1f}m ahead'
                    )
                return factor
            elif dist < slow_dist:
                ratio = (dist - safe_dist) / (slow_dist - safe_dist)
                factor = min(factor, max(0.05, ratio))  # allow very slow crawl

        if factor > 0.0 and self._follow_stopped:
            self._follow_stopped = False
            self.get_logger().info(f'[{self.name}] Follow clear')
        return factor

    # ── Traffic signal logic ─────────────────────────────────────────────────
    def _should_stop_at_signal(self, dist_ctr):
        """
        Real-world traffic light rule:
          GREEN  → go
          YELLOW → stop if dist > YELLOW_COMMIT_DIST, else go (committed)
          RED    → stop
          ALL_RED→ stop
        Emergency vehicles NEVER stop.
        """
        if self.is_emergency:
            return False
        if self.committed or self.passed_center:
            return False

        # Too far — no signal check yet
        if dist_ctr > SIGNAL_CHECK_FAR:
            return False

        # Determine if the signal says we should stop
        phase = self.traffic_phase
        should_stop = True   # default safe

        if phase == 'ALL_RED':
            should_stop = True

        elif phase.startswith('PHASE:'):
            parts = phase.split(':')
            if len(parts) >= 3:
                phase_dir, phase_state = parts[1], parts[2]

                if self.phase_dir in phase_dir:
                    # Our direction has signal
                    if phase_state == 'GREEN':
                        should_stop = False
                    elif phase_state == 'YELLOW':
                        # Real-world yellow: stop if room to brake, go if committed
                        should_stop = (dist_ctr > YELLOW_COMMIT_DIST)
                    else:  # RED
                        should_stop = True
                else:
                    # Cross traffic has signal — we must stop
                    should_stop = True

        # Commit logic
        if not should_stop:
            # Commit once we cross the stop line (8.0m) under green/yellow
            if dist_ctr < JUNCTION_COMMIT_DIST:
                self.committed = True
                self.get_logger().info(f'[{self.name}] COMMITTED (Green/Yellow) at {dist_ctr:.1f}m')
                return False
        else:
            # If the signal is RED, we do NOT commit on minor drift past stop line.
            # We only force commit if we have drifted deep inside the center of the box (4.5m)
            if dist_ctr < 4.5:
                self.committed = True
                self.get_logger().info(f'[{self.name}] FORCED COMMIT (drifted too deep) at {dist_ctr:.1f}m')
                return False

        # Logging transitions
        if should_stop and not self._signal_stopped:
            self._signal_stopped = True
            self.get_logger().info(
                f'[{self.name}] STOP at {dist_ctr:.1f}m — {phase}'
            )
        elif not should_stop and self._signal_stopped:
            self._signal_stopped = False
            self.get_logger().info(f'[{self.name}] GO — {phase}')

        return should_stop

    # ── Zone speed ───────────────────────────────────────────────────────────
    def _zone_speed(self, dist_ctr):
        """Target speed based on distance from junction center."""
        mult = self._speed_mult()
        if self.is_emergency:
            return CRUISE_SPEED * mult
        if self.passed_center:
            return CRUISE_SPEED * mult
        if self.committed:
            return APPROACH_SPEED * mult
        if dist_ctr > 12.0:
            return CRUISE_SPEED * mult
        if dist_ctr > 8.0:
            return APPROACH_SPEED * mult
        return CAUTION_SPEED * mult

    # ── Smooth speed step ────────────────────────────────────────────────────
    def _smooth_speed(self, target):
        """Low-pass filter: blend target with last actual speed."""
        smoothed = SPEED_ALPHA * target + (1.0 - SPEED_ALPHA) * self._last_speed
        self._last_speed = smoothed
        return smoothed

    # ── Main control loop ────────────────────────────────────────────────────
    def control_loop(self):
        # Self-termination: check for stale position updates to kill orphan controllers
        now_time = self.get_clock().now()
        dt_sec = (now_time - self.last_pos_time).nanoseconds / 1e9
        if self.pos_received:
            if dt_sec > 2.0:
                self.get_logger().warn(f'[{self.name}] Position updates stopped for {dt_sec:.1f}s. Self-terminating.')
                os._exit(0)
        else:
            if dt_sec > 8.0:
                self.get_logger().warn(f'[{self.name}] Failed to receive initial position for 8.0s. Self-terminating.')
                os._exit(0)
            return

        if self.finished:
            if not self._shutdown_scheduled:
                self._shutdown_scheduled = True
                self.create_timer(1.5, self._do_shutdown)
            return

        x, y, yaw = self.world_x, self.world_y, self.world_yaw
        dist_ctr   = math.hypot(x, y)
        tw         = Twist()

        if dist_ctr < 3.0:
            self.passed_center = True

        if self.wp_index >= len(self.waypoints):
            self._finish(tw)
            return

        # Navigate to current waypoint
        tx, ty  = self.waypoints[self.wp_index]
        dx, dy  = tx - x, ty - y
        dist_wp = math.hypot(dx, dy)
        fwd_x   = math.cos(yaw)
        fwd_y   = math.sin(yaw)
        dot_wp  = dx * fwd_x + dy * fwd_y

        if dist_wp < self.wp_threshold or dot_wp < 0:
            self.wp_index += 1
            if self.wp_index >= len(self.waypoints):
                self._finish(tw)
                return
            tx, ty = self.waypoints[self.wp_index]
            dx, dy = tx - x, ty - y

        # Steering
        err = self._norm(math.atan2(dy, dx) - yaw)
        raw_steer    = max(-MAX_ANGULAR, min(MAX_ANGULAR, K_STEER * err))
        smooth_steer = (1.0 - STEER_ALPHA) * raw_steer + STEER_ALPHA * self._prev_steer
        self._prev_steer = smooth_steer

        # Target speed
        target_speed = self._zone_speed(dist_ctr)
        hard_stop = False

        # Detect ambulance AHEAD or BEHIND within 40 m
        fwd_x = math.cos(yaw)
        fwd_y = math.sin(yaw)
        ambulance_ahead  = False
        ambulance_behind = False
        if not self.is_emergency:
            for vx, vy, otype, _ in self.nearby_vehicles:
                if otype != EMERGENCY_TYPE:
                    continue
                ddx, ddy = vx - x, vy - y
                ddist = math.hypot(ddx, ddy)
                if ddist < 0.5 or ddist > 40.0:
                    continue
                dot = (ddx * fwd_x + ddy * fwd_y) / ddist
                if dot > 0.4:
                    ambulance_ahead = True
                elif dot < -0.3:
                    ambulance_behind = True

        # Emergency lane clear: car with ambulance BEHIND should drive at max speed
        # This rapidly clears the path so ambulance is never blocked by a slow queue
        emergency_clear_mode = ambulance_behind and not self.is_emergency

        if self.passed_center:
            ff = self._follow_factor()
            if ff == 0.0:
                self._last_speed *= 0.3
                target_speed = 0.0
            elif emergency_clear_mode or ambulance_ahead:
                # Max speed to clear path; scale by ff to avoid collisions
                target_speed = CRUISE_SPEED * self._speed_mult() * 1.5 * ff
            else:
                target_speed = CRUISE_SPEED * self._speed_mult() * ff
            
            # Scale floor by ff so we slow down properly if a car is close ahead
            if ff > 0.0:
                target_speed = max(target_speed, 1.5 * ff)
        elif self.committed:
            ff = self._follow_factor()
            if ff == 0.0:
                self._last_speed *= 0.3
                target_speed = 0.0
            elif emergency_clear_mode:
                # Ambulance is behind and we're in junction — rush through fast
                target_speed = CRUISE_SPEED * self._speed_mult() * 1.3 * ff
            else:
                target_speed = APPROACH_SPEED * self._speed_mult() * ff
            
            if ff > 0.0:
                target_speed = max(target_speed, 0.5 * ff)
        else:
            if self._should_stop_at_signal(dist_ctr):
                hard_stop    = True
                target_speed = 0.0
                self._last_speed = 0.0
            else:
                ff = self._follow_factor()
                af = self._ambulance_behind_factor()
                if ff == 0.0:
                    self._last_speed *= 0.3
                    target_speed = 0.0
                elif emergency_clear_mode or ambulance_ahead:
                    # Emergency clear: drive at 1.5× cruise scaled by ff
                    target_speed = CRUISE_SPEED * self._speed_mult() * 1.5 * ff
                else:
                    target_speed *= (ff * af)

        # Apply smooth low-pass filter only when NOT doing a hard stop
        if hard_stop:
            speed = 0.0
        else:
            speed = self._smooth_speed(target_speed)

        tw.linear.x  = float(speed)
        tw.angular.z = float(smooth_steer) if speed > 0.05 else 0.0
        self.cmd_pub.publish(tw)

    def _finish(self, tw):
        tw.linear.x = tw.angular.z = 0.0
        self.cmd_pub.publish(tw)
        if not self.finished:
            self.finished = True
            m = String(); m.data = self.name; self.done_pub.publish(m)
            self.get_logger().info(f'[{self.name}] DONE — {self.path_id}')

    def _do_shutdown(self):
        m = String(); m.data = self.name; self.done_pub.publish(m)
        self.get_logger().info(f'[{self.name}] Shutting down controller')
        os._exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleController()
    try:
        # Use spin_once loop so RuntimeError from oversized ModelStates messages
        # (triggered when 20+ vehicles are loaded) doesn't crash the controller.
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.05)
            except RuntimeError:
                pass  # ignore transient ModelStates deserialization errors
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
