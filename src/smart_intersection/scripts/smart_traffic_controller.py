#!/usr/bin/env python3
"""
smart_traffic_controller.py — AI-based adaptive traffic control
===============================================================
Features:
  1. CV-based traffic analysis integration (density, queue, vehicle types)
  2. Graduated green timing for paired phases (NS / EW)
  3. Paired phases (North+South, East+West) for straight-only intersection
  4. Graduated emergency vehicle priority per pair
  5. Lamp state tracking to prevent async race conditions
  6. Real-time continuous traffic condition updates
"""

import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnEntity, SetEntityState

PRIORITY_WEIGHT = {'ambulance': 5, 'fire_engine': 4, 'police': 3, 'bus': 1, 'car': 0}

# Graduated green timing by density level
GREEN_BY_DENSITY = {
    'LOW':       6.0,
    'MEDIUM':   12.0,
    'HIGH':     18.0,
    'VERY_HIGH': 25.0,
}
BASE_GREEN_TIME  = 8.0
DENSITY_BONUS    = 2.0
MAX_GREEN_TIME   = 25.0
YELLOW_TIME      = 3.0
ALL_RED_TIME     = 2.0
STARTUP_RED_TIME = 20.0

DENSITY_MIN_DIST  =  5.0
DENSITY_MAX_DIST  = 35.0
DENSITY_MAX_SPEED =  1.0
DENSITY_THRESHOLD =  1

CLOCKWISE_ORDER = ['N', 'E', 'S', 'W']
PHASES = ['NS', 'EW']
SPAWN_TO_DIR    = {'A': 'N', 'C': 'S', 'B': 'E', 'D': 'W'}

# Traffic light pole world positions (from intersection.world include poses)
POLE_POSITIONS = {
    'N': ( 4.5,  5.5),
    'S': (-4.5, -5.5),
    'E': ( 5.5, -4.5),
    'W': (-5.5,  4.5),
}

def _lamp_xy(direction, pole_x, pole_y):
    """Offset lamp 0.175m from pole toward incoming traffic."""
    if direction == 'N': return pole_x, pole_y + 0.175
    if direction == 'S': return pole_x, pole_y - 0.175
    if direction == 'E': return pole_x + 0.175, pole_y
    if direction == 'W': return pole_x - 0.175, pole_y
    return pole_x, pole_y

LIGHT_COORDS = {
    d: {'x': _lamp_xy(d, *POLE_POSITIONS[d])[0],
        'y': _lamp_xy(d, *POLE_POSITIONS[d])[1]}
    for d in POLE_POSITIONS
}

# Lamp z positions inside housing
Z_OFFSETS = {'red': 3.55, 'yel': 3.15, 'grn': 2.75}
Z_HIDDEN  = -10.0


class SmartTrafficController(Node):

    def __init__(self):
        super().__init__('smart_traffic_controller')

        self.phase_pub = self.create_publisher(String, '/traffic_phase',     10)
        self.timer_pub = self.create_publisher(String, '/signal_timers',     10)
        self.pos_pub   = self.create_publisher(String, '/vehicle_positions', 10)
        self.lock_pub  = self.create_publisher(String, '/intersection_lock', 10)

        # Allow startup_red_time to be overridden via launch parameter
        self.declare_parameter('startup_red_time', STARTUP_RED_TIME)
        self.actual_startup_red = self.get_parameter('startup_red_time').value

        self.create_subscription(String, '/vehicle_registry', self.registry_cb, 10)
        self.create_subscription(String, '/vehicle_counts', self.camera_counts_cb, 10)
        self.create_subscription(String, '/traffic_analysis', self.traffic_analysis_cb, 10)
        self.create_subscription(String, '/emergency_request', self.emergency_cb, 10)
        self.camera_density = {d: 0 for d in CLOCKWISE_ORDER}
        self.cv_density_level = {d: 'LOW' for d in CLOCKWISE_ORDER}
        self.cv_queue_length = {d: 0 for d in CLOCKWISE_ORDER}
        self.cv_emergency = {d: False for d in CLOCKWISE_ORDER}

        self.spawn_cli = self.create_client(SpawnEntity,    '/spawn_entity')
        self.state_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.get_logger().info('Waiting for Gazebo services...')
        self.spawn_cli.wait_for_service(timeout_sec=30.0)
        self.state_cli.wait_for_service(timeout_sec=30.0)
        self.get_logger().info('Services ready.')

        self.vehicles   = {}
        self.vehicle_ts = {}

        self.current_phase  = None
        self.current_colour = 'RED'
        self.state_start    = self.get_clock().now()
        self.startup        = True
        self.current_green_duration = BASE_GREEN_TIME

        self.density    = {d: 0 for d in CLOCKWISE_ORDER}
        self.wait_start = {p: self.get_clock().now() for p in PHASES}

        self.lock_vehicle = None
        self.lock_time    = None
        self.locked       = False

        # Emergency override state — graduated approach
        self.emergency_active   = False
        self.emergency_axis     = None   # 'NS','EW'
        self.emergency_vehicle  = None
        self.emergency_cooldown = 0.0    # seconds remaining before another override
        self.emergency_state    = None   # 'APPROACH', 'CLOSE', 'OVERRIDE'

        # Lamp state tracking
        self._lamp_z = {}
        for d in POLE_POSITIONS:
            for c in ('red', 'yel', 'grn'):
                self._lamp_z[f'lamp_{d}_{c}'] = Z_HIDDEN

        self.lamps_ready = False
        self._spawn_all_lamps()

        self.create_timer(0.5, self.tick)
        self.create_timer(0.3, self.publish_positions)
        self.create_timer(0.3, self.publish_lock)
        self.create_timer(1.0, self.update_density)

        self.get_logger().info(
            f'Traffic Controller READY — {self.actual_startup_red}s startup ALL_RED'
        )

    # ── Lamp helpers ───────────────────────────────────────────────────────────
    def _sphere_sdf(self, name, r, g, b):
        return (f"<?xml version='1.0'?><sdf version='1.6'>"
                f"<model name='{name}'><static>true</static>"
                f"<link name='link'><visual name='v'>"
                f"<geometry><sphere><radius>0.19</radius></sphere></geometry>"
                f"<material>"
                f"<ambient>{r} {g} {b} 1</ambient>"
                f"<diffuse>{r} {g} {b} 1</diffuse>"
                f"<emissive>{r:.1f} {g:.1f} {b:.1f} 1</emissive>"
                f"</material></visual></link></model></sdf>")

    def _spawn_all_lamps(self):
        colours = [('red', 1.0, 0.0, 0.0),
                   ('yel', 1.0, 0.75, 0.0),
                   ('grn', 0.0, 0.9, 0.0)]
        self._spawn_queue = []
        for d in POLE_POSITIONS:
            for cname, r, g, b in colours:
                req = SpawnEntity.Request()
                req.name = f'lamp_{d}_{cname}'
                req.xml  = self._sphere_sdf(f'lamp_{d}_{cname}', r, g, b)
                req.robot_namespace = '/gazebo'
                req.initial_pose.position.z = Z_HIDDEN
                self._spawn_queue.append(req)
        # Process spawn queue sequentially every 0.2s
        self._spawn_timer = self.create_timer(0.2, self._process_spawn_queue)

    def _process_spawn_queue(self):
        if self._spawn_queue:
            req = self._spawn_queue.pop(0)
            self.spawn_cli.call_async(req)
        else:
            self._spawn_timer.cancel()
            self._lamp_timer = self.create_timer(3.0, self._lamps_ready_cb)

    def _lamps_ready_cb(self):
        if self.lamps_ready:
            return
        self.lamps_ready = True
        if hasattr(self, '_lamp_timer') and self._lamp_timer is not None:
            self._lamp_timer.cancel()
            
        # Apply the current target state now that spheres exist
        if getattr(self, 'current_colour', 'ALL_RED') == 'GREEN':
            self._apply_green(self.current_phase)
        elif getattr(self, 'current_colour', '') == 'YELLOW':
            self._apply_yellow(self.current_phase)
        else:
            self._set_all_red_immediate()
            
        self.get_logger().info('Lamps ready — synced to active phase')

    def _move_lamp_if_changed(self, name, x, y, target_z):
        """Only send SetEntityState if z actually needs to change."""
        if not self.lamps_ready:
            return
        current_z = self._lamp_z.get(name, Z_HIDDEN)
        if abs(current_z - target_z) < 0.01:
            return
        self._lamp_z[name] = target_z
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(target_z)
        self.state_cli.call_async(req)

    def _set_pole(self, d, colour):
        """Show exactly ONE lamp on pole d."""
        c = LIGHT_COORDS[d]
        for lamp_colour in ('red', 'yel', 'grn'):
            target_z = Z_OFFSETS[lamp_colour] if lamp_colour == colour else Z_HIDDEN
            self._move_lamp_if_changed(f'lamp_{d}_{lamp_colour}', c['x'], c['y'], target_z)

    def _set_all_red_immediate(self):
        """Set all 4 poles to RED at once."""
        for d in POLE_POSITIONS:
            self._set_pole(d, 'red')

    def _apply_green(self, phase):
        """phase -> green, all others -> red."""
        for d in POLE_POSITIONS:
            if phase == 'NS' and d in ('N', 'S'):
                self._set_pole(d, 'grn')
            elif phase == 'EW' and d in ('E', 'W'):
                self._set_pole(d, 'grn')
            else:
                self._set_pole(d, 'red')

    def _apply_yellow(self, phase):
        """phase -> yellow, all others -> red."""
        for d in POLE_POSITIONS:
            if phase == 'NS' and d in ('N', 'S'):
                self._set_pole(d, 'yel')
            elif phase == 'EW' and d in ('E', 'W'):
                self._set_pole(d, 'yel')
            else:
                self._set_pole(d, 'red')

    def _apply_all_red(self):
        for d in POLE_POSITIONS:
            self._set_pole(d, 'red')

    # ── Vehicle registry ───────────────────────────────────────────────────────
    def registry_cb(self, msg: String):
        parts = msg.data.split(':')
        if len(parts) < 8:
            return
        name, vtype, spawn_dir = parts[0], parts[1], parts[2]
        direction = SPAWN_TO_DIR.get(spawn_dir, spawn_dir)
        try:
            self.vehicles[name] = {
                'type':  vtype, 'dir': direction,
                'x':     float(parts[3]), 'y': float(parts[4]),
                'speed': float(parts[5]), 'dist': float(parts[6]),
            }
            self.vehicle_ts[name] = self.get_clock().now()
        except (ValueError, IndexError):
            pass

    def camera_counts_cb(self, msg: String):
        try:
            direction, count_str = msg.data.split(':')
            count = int(count_str)
            if direction in self.camera_density:
                self.camera_density[direction] = count
        except Exception:
            pass

    def traffic_analysis_cb(self, msg: String):
        """Process JSON traffic analysis from enhanced vehicle_counter."""
        try:
            data = json.loads(msg.data)
            d = data.get('direction', '')
            if d in self.cv_density_level:
                self.cv_density_level[d] = data.get('density', 'LOW')
                self.cv_queue_length[d] = data.get('queue_length', 0)
                self.cv_emergency[d] = data.get('emergency_detected', False)
                count = data.get('vehicle_count', 0)
                self.camera_density[d] = max(self.camera_density.get(d, 0), count)
        except (json.JSONDecodeError, Exception):
            pass

    # ── Emergency override — graduated approach ────────────────────────────────
    def emergency_cb(self, msg: String):
        parts = msg.data.split(':')
        if len(parts) < 2:
            return

        action = parts[0]

        if action in ('EMERGENCY', 'EMERGENCY_APPROACH', 'EMERGENCY_CLOSE') and len(parts) >= 6:
            axis = parts[1]   # 'NS' or 'EW'
            name = parts[2]
            vtype = parts[3]
            dist = float(parts[5]) if len(parts) > 5 else 20.0

            if self.emergency_cooldown > 0:
                self.get_logger().info(f'Emergency cooldown active — ignoring {name}')
                return

            em_axis = axis

            self.emergency_active   = True
            self.emergency_axis     = em_axis
            self.emergency_vehicle  = name
            self.emergency_cooldown = 5.0

            # Graduated response based on distance
            if dist > 15.0:
                self.emergency_state = 'APPROACH'
                if self.current_phase == em_axis and self.current_colour == 'GREEN':
                    self.current_green_duration = min(self.current_green_duration + 5.0, MAX_GREEN_TIME + 5.0)
                    self.get_logger().warn(f'EMERGENCY APPROACH → extending {em_axis} GREEN for {vtype} ({name}) at {dist:.0f}m')
                else:
                    self.current_phase  = em_axis
                    self.current_colour = 'EMERGENCY'
                    self.state_start    = self.get_clock().now()
                    self._apply_green(em_axis)
                    self.get_logger().warn(f'EMERGENCY APPROACH → {em_axis} GREEN for {vtype} ({name})')
            elif dist > 5.0:
                self.emergency_state = 'CLOSE'
                self.current_phase  = em_axis
                self.current_colour = 'EMERGENCY'
                self.state_start    = self.get_clock().now()
                self._apply_green(em_axis)
                self.get_logger().warn(f'EMERGENCY CLOSE → {em_axis} GREEN HOLD for {vtype} ({name})')
            else:
                self.emergency_state = 'OVERRIDE'
                self.current_phase  = em_axis
                self.current_colour = 'EMERGENCY'
                self.state_start    = self.get_clock().now()
                self._apply_green(em_axis)
                self.get_logger().warn(f'EMERGENCY OVERRIDE → {em_axis} FULL GREEN for {vtype} ({name})')

        elif action == 'CLEAR' and len(parts) >= 3:
            name = parts[2]
            if self.emergency_active and self.emergency_vehicle == name:
                self.get_logger().info(f'Emergency CLEAR for {name} — transition normal')
                self.emergency_active  = False
                self.emergency_axis    = None
                self.emergency_vehicle = None
                self.emergency_state   = None
                self.current_colour = 'YELLOW'
                self.state_start    = self.get_clock().now()
                self._apply_yellow(self.current_phase)

    def _prune_stale(self):
        now   = self.get_clock().now()
        stale = [n for n, t in self.vehicle_ts.items() if (now - t).nanoseconds > 5_000_000_000]
        for n in stale:
            self.vehicles.pop(n, None)
            self.vehicle_ts.pop(n, None)
            if self.lock_vehicle == n:
                self.locked       = False
                self.lock_vehicle = None
                self.lock_time    = None

    # ── Density counting ───────────────────────────────────────────────────────
    def update_density(self):
        counts = {d: 0 for d in CLOCKWISE_ORDER}
        for v in self.vehicles.values():
            d = v['dir']
            if d in counts:
                if DENSITY_MIN_DIST < v['dist'] < DENSITY_MAX_DIST:
                    if v['speed'] < DENSITY_MAX_SPEED:
                        counts[d] += 1
        for d in CLOCKWISE_ORDER:
            cam_count = self.camera_density.get(d, 0)
            counts[d] = max(counts[d], cam_count)
        self.density = counts
        if sum(counts.values()) > 0:
            self.get_logger().info(f'Density: N={counts["N"]} S={counts["S"]} | E={counts["E"]} W={counts["W"]}')

    def _pick_next_phase(self, exclude=None):
        if exclude == 'NS': return 'EW'
        if exclude == 'EW': return 'NS'
        ns_d = self.density.get('N',0) + self.density.get('S',0)
        ew_d = self.density.get('E',0) + self.density.get('W',0)
        ns_s = ns_d * 10.0 + (self.get_clock().now() - self.wait_start['NS']).nanoseconds / 1e9 * 0.1
        ew_s = ew_d * 10.0 + (self.get_clock().now() - self.wait_start['EW']).nanoseconds / 1e9 * 0.1
        return 'NS' if ns_s >= ew_s else 'EW'

    def _green_duration_for(self, phase):
        dirs = ('N', 'S') if phase == 'NS' else ('E', 'W')
        d1, d2 = dirs
        levels = {'LOW': 1, 'MEDIUM': 2, 'HIGH': 3, 'VERY_HIGH': 4}
        l1 = levels.get(self.cv_density_level.get(d1, 'LOW'), 1)
        l2 = levels.get(self.cv_density_level.get(d2, 'LOW'), 1)
        m = max(l1, l2)
        max_level = 'VERY_HIGH' if m==4 else 'HIGH' if m==3 else 'MEDIUM' if m==2 else 'LOW'
        cv_time = GREEN_BY_DENSITY.get(max_level, BASE_GREEN_TIME)

        waiting = max(self.density.get(d1, 0), self.density.get(d2, 0))
        count_time = min(BASE_GREEN_TIME + max(0, waiting - DENSITY_THRESHOLD) * DENSITY_BONUS, MAX_GREEN_TIME)

        queue_bonus = min(max(self.cv_queue_length.get(d1, 0), self.cv_queue_length.get(d2, 0)) * 1.0, 5.0)

        return min(max(cv_time, count_time) + queue_bonus, MAX_GREEN_TIME)

    # ── Intersection lock ──────────────────────────────────────────────────────
    def _update_lock(self):
        if self.startup:
            return
        now    = self.get_clock().now()
        center = [(n, v) for n, v in self.vehicles.items() if v['dist'] < 2.0]
        if self.locked:
            holder = self.vehicles.get(self.lock_vehicle)
            if holder is None or holder['dist'] > 4.0:
                self.locked = False; self.lock_vehicle = None; self.lock_time = None
            elif self.lock_time:
                if (now - self.lock_time).nanoseconds / 1e9 > 10.0:
                    self.get_logger().warn('Gridlock timeout — force unlock')
                    self.locked = False; self.lock_vehicle = None; self.lock_time = None
        elif center:
            self.locked = True; self.lock_vehicle = center[0][0]; self.lock_time = now

    # ── Publishers ─────────────────────────────────────────────────────────────
    def publish_positions(self):
        entries = [f"{n}:{v['x']:.2f}:{v['y']:.2f}:{v['type']}" for n, v in self.vehicles.items()]
        m = String(); m.data = ','.join(entries); self.pos_pub.publish(m)

    def publish_lock(self):
        m = String()
        m.data = f'LOCKED:{self.lock_vehicle}' if self.locked else 'UNLOCKED'
        self.lock_pub.publish(m)

    def _publish_timers(self, remaining_for_current):
        timers = {d: 0 for d in CLOCKWISE_ORDER}
        if self.current_phase and self.current_colour == 'GREEN':
            rem = max(0, int(remaining_for_current))
            if self.current_phase == 'NS': timers['N'] = rem; timers['S'] = rem
            if self.current_phase == 'EW': timers['E'] = rem; timers['W'] = rem
        m = String()
        m.data = f"N:{timers['N']:02d},E:{timers['E']:02d},S:{timers['S']:02d},W:{timers['W']:02d}"
        self.timer_pub.publish(m)

    def _publish_phase(self, phase_str):
        m = String(); m.data = phase_str; self.phase_pub.publish(m)

    # ── Main tick ──────────────────────────────────────────────────────────────
    def tick(self):
        self._prune_stale()
        self._update_lock()

        now     = self.get_clock().now()
        elapsed = (now - self.state_start).nanoseconds / 1e9

        if self.emergency_cooldown > 0:
            self.emergency_cooldown = max(0, self.emergency_cooldown - 0.5)

        if self.startup:
            self._publish_phase('ALL_RED')
            self._publish_timers(0)
            if elapsed >= self.actual_startup_red:
                self.startup     = False
                self.current_phase = self._pick_next_phase()
                self._start_green(now)
            return

        if self.current_colour == 'GREEN':
            remaining = self.current_green_duration - elapsed
            self._publish_phase(f'PHASE:{self.current_phase}:GREEN')
            self._publish_timers(remaining)
            if elapsed >= self.current_green_duration:
                self.current_colour = 'YELLOW'
                self.state_start    = now
                self._apply_yellow(self.current_phase)
                self.get_logger().info(f'{self.current_phase} → YELLOW')

        elif self.current_colour == 'EMERGENCY':
            self._publish_phase(f'PHASE:{self.current_phase}:GREEN')
            self._publish_timers(0)
            if elapsed >= 30.0:
                self.get_logger().warn('Emergency timeout 30s — force resume')
                self.emergency_active  = False
                self.emergency_axis    = None
                self.emergency_vehicle = None
                self.current_colour    = 'YELLOW'
                self.state_start       = now
                self._apply_yellow(self.current_phase)

        elif self.current_colour == 'YELLOW':
            self._publish_phase(f'PHASE:{self.current_phase}:YELLOW')
            self._publish_timers(0)
            if elapsed >= YELLOW_TIME:
                self.current_colour = 'ALL_RED'
                self.state_start    = now
                self._apply_all_red()
                self.get_logger().info(f'{self.current_phase} → ALL_RED buffer')

        elif self.current_colour == 'ALL_RED':
            self._publish_phase('ALL_RED')
            self._publish_timers(0)
            if elapsed >= ALL_RED_TIME:
                prev = self.current_phase
                self.current_phase = self._pick_next_phase(exclude=prev)
                self._start_green(now)

    def _start_green(self, now):
        self.current_colour         = 'GREEN'
        self.state_start            = now
        self.current_green_duration = self._green_duration_for(self.current_phase)
        self._apply_green(self.current_phase)
        self.wait_start[self.current_phase] = now
        ns_d = self.density.get('N',0) + self.density.get('S',0)
        ew_d = self.density.get('E',0) + self.density.get('W',0)
        self.get_logger().info(f'{self.current_phase} → GREEN for {self.current_green_duration:.1f}s (queued: NS={ns_d} EW={ew_d})')


def main(args=None):
    rclpy.init(args=args)
    node = SmartTrafficController()
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
