#!/usr/bin/env python3
"""
vehicle_spawner.py — Cars (multi-color) + Ambulance only
=========================================================
Bugs fixed:
  1. spawn_cooldown per direction prevents duplicate spawns at same spot
     even when Gazebo silently fails to create vehicle
  2. Faster spawn interval + higher per-tick limit = no empty road
  3. Reduced LANE_CLEAR_DIST so lanes open up sooner
  4. Aggressive refill: if below minimum, spawn up to 4 per tick
  5. deleting set cleared with timeout (10s) so stale entries don't block
  6. Ambulance cycle: after ambulance clears, counter reset → next in 4 cars
"""

import math
import os
import time
import subprocess

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

# ---------------------------------------------------------------------------
# Spawn points  (direction → x, y, yaw)
# ---------------------------------------------------------------------------
SPAWN_POINTS = {
    'A': {'x':  2.0, 'y':  45.0, 'yaw': -math.pi / 2},
    'B': {'x': 45.0, 'y': -2.0,  'yaw':  math.pi},
    'C': {'x': -2.0, 'y': -45.0, 'yaw':  math.pi / 2},
    'D': {'x':-45.0, 'y':  2.0,  'yaw':  0.0},
}

PATHS_FROM = {
    'A': 'A_to_C',
    'B': 'B_to_D',
    'C': 'C_to_A',
    'D': 'D_to_B',
}

# ---------------------------------------------------------------------------
# 10-color palette — injected into car SDF at spawn time
# ---------------------------------------------------------------------------
CAR_COLORS = [
    ('0.88 0.12 0.12', 'Red'),
    ('0.10 0.30 0.88', 'Blue'),
    ('0.10 0.72 0.22', 'Green'),
    ('0.95 0.45 0.05', 'Orange'),
    ('0.55 0.12 0.82', 'Purple'),
    ('0.05 0.68 0.62', 'Teal'),
    ('0.88 0.82 0.08', 'Yellow'),
    ('0.90 0.25 0.55', 'Pink'),
    ('0.70 0.70 0.78', 'Silver'),
    ('0.50 0.08 0.12', 'Maroon'),
]
CAR_ORIGINAL_COLOR = '0.96 0.96 0.86'   # original ivory in car/model.sdf

# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------
EMERGENCY_TYPE    = 'ambulance'
EMERGENCY_CHANCE  = 4      # spawn ambulance every N cars (if no active emergency)

MIN_PER_DIRECTION = 3      # keep at least this many per road
MAX_ACTIVE        = 20     # hard cap on simultaneous vehicles
SPAWN_INTERVAL    = 1.5    # seconds between spawn ticks (faster = fuller roads)
SPAWN_Z           = 0.15

# Minimum gap from spawn point before next vehicle may spawn from that direction
LANE_CLEAR_DIST   = 8.0    # metres (reduced from 15 → lanes open faster)

# Per-direction cooldown: seconds after spawning before same direction may spawn again
DIR_COOLDOWN      = 4.0    # prevents overlapping spawns when Gazebo is slow

# How many vehicles to spawn per tick when road is below minimum
REFILL_BATCH      = 3      # up to 3 per tick during refill phase

# Stale deleting timeout — if Gazebo doesn't confirm deletion in N seconds, force-clear
DELETE_TIMEOUT    = 10.0


class VehicleSpawner(Node):
    def __init__(self):
        super().__init__('vehicle_spawner')

        self.spawn_cli  = self.create_client(SpawnEntity,  '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        # Direct emergency publisher — fires immediately at ambulance spawn
        # so the traffic controller switches to green BEFORE the ambulance moves
        self.em_pub = self.create_publisher(String, '/emergency_request', 10)

        self.get_logger().info('VehicleSpawner: waiting for Gazebo services...')
        self.spawn_cli.wait_for_service(timeout_sec=30.0)
        self.get_logger().info('VehicleSpawner: ready.')

        self.pkg_path = get_package_share_directory('smart_intersection')

        # Load SDF templates once
        with open(os.path.join(self.pkg_path, 'models', 'car', 'model.sdf')) as f:
            self.car_sdf_template = f.read()
        with open(os.path.join(self.pkg_path, 'models', 'ambulance', 'model.sdf')) as f:
            self.ambulance_sdf = f.read()

        self.count      = 0
        self.active     = {}      # name → {proc, path, dir, type, color, spawn_time}
        self.positions  = {}      # name → (x, y)
        self.done_vehicles = set()
        self.deleting   = {}      # name → timestamp when delete was requested

        # Per-direction spawn cooldown  (direction → last spawn time)
        self.last_spawn_time = {'A': 0.0, 'B': 0.0, 'C': 0.0, 'D': 0.0}

        # Emergency tracking
        self.active_emergency      = None   # name of live ambulance, or None
        self.regular_spawn_counter = 0      # cars spawned since last ambulance

        # Color round-robin
        self.color_index = 0

        self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_states_cb, 10
        )
        self.create_subscription(String, '/vehicle_done', self.done_cb, 10)

        self.create_timer(SPAWN_INTERVAL, self.try_spawn)
        self.create_timer(2.0,            self.cleanup)

        self.get_logger().info(
            f'VehicleSpawner ready — '
            f'min {MIN_PER_DIRECTION}/road, max {MAX_ACTIVE}, '
            f'ambulance every {EMERGENCY_CHANCE} cars, '
            f'interval={SPAWN_INTERVAL}s'
        )

    # ── Gazebo position callback ─────────────────────────────────────────────
    def model_states_cb(self, msg: ModelStates):
        try:
            for i, name in enumerate(msg.name):
                if name.startswith('vehicle_'):
                    p = msg.pose[i].position
                    self.positions[name] = (p.x, p.y)
        except (RuntimeError, IndexError):
            pass

    def done_cb(self, msg: String):
        self.done_vehicles.add(msg.data)

    # ── Lane clear check ────────────────────────────────────────────────────
    def _lane_clear(self, direction):
        """
        True if no known vehicle is within LANE_CLEAR_DIST of spawn point
        AND the per-direction cooldown has expired.
        """
        now = time.monotonic()
        if now - self.last_spawn_time[direction] < DIR_COOLDOWN:
            return False   # cooldown not expired

        sp = SPAWN_POINTS[direction]
        sx, sy = sp['x'], sp['y']

        # Check positions reported by Gazebo
        for name, (vx, vy) in list(self.positions.items()):
            if math.hypot(vx - sx, vy - sy) < LANE_CLEAR_DIST:
                return False

        # Also check recently spawned vehicles that may not have positions yet
        # (covers the case where Gazebo spawn is slow to report)
        for name, info in self.active.items():
            if info['dir'] == direction:
                age = now - info['spawn_time']
                if age < DIR_COOLDOWN:   # hasn't had time to move away yet
                    return False

        return True

    # ── Vehicle selection ────────────────────────────────────────────────────
    def _pick_vehicle(self):
        """Return (vtype, color_rgb, color_name)."""
        # Spawn ambulance if due and no ambulance active
        if (self.active_emergency is None and
                self.regular_spawn_counter >= EMERGENCY_CHANCE):
            self.regular_spawn_counter = 0
            return (EMERGENCY_TYPE, None, None)

        # Regular car with round-robin color
        rgb, cname = CAR_COLORS[self.color_index % len(CAR_COLORS)]
        self.color_index += 1
        self.regular_spawn_counter += 1
        return ('car', rgb, cname)

    def _build_car_sdf(self, color_rgb):
        return self.car_sdf_template.replace(CAR_ORIGINAL_COLOR, color_rgb)

    # ── Direction count ──────────────────────────────────────────────────────
    def _count_per_direction(self):
        counts = {'A': 0, 'B': 0, 'C': 0, 'D': 0}
        for info in self.active.values():
            d = info.get('dir', '')
            if d in counts:
                counts[d] += 1
        return counts

    # ── Spawn tick ───────────────────────────────────────────────────────────
    def try_spawn(self):
        if len(self.active) >= MAX_ACTIVE:
            return

        dir_counts = self._count_per_direction()
        spawned    = 0
        max_batch  = REFILL_BATCH

        # Priority pass: fill directions below minimum first
        for d in ['A', 'B', 'C', 'D']:
            if spawned >= max_batch:
                break
            if len(self.active) >= MAX_ACTIVE:
                break
            if dir_counts[d] < MIN_PER_DIRECTION and self._lane_clear(d):
                self._do_spawn(d)
                dir_counts[d] += 1
                spawned += 1

        # If all directions at minimum and still have budget: round-robin extras
        if spawned == 0:
            dirs = ['A', 'B', 'C', 'D']
            for d in dirs:
                if len(self.active) >= MAX_ACTIVE:
                    break
                if self._lane_clear(d):
                    self._do_spawn(d)
                    spawned += 1
                    break   # one extra per tick when fully stocked

        if spawned:
            self.get_logger().info(
                f'[spawner] +{spawned} spawned  '
                f'A={dir_counts["A"]} B={dir_counts["B"]} '
                f'C={dir_counts["C"]} D={dir_counts["D"]}  '
                f'total={len(self.active)}'
            )

    def _do_spawn(self, direction):
        self.count += 1
        name = f'vehicle_{self.count}'
        vtype, color_rgb, color_name = self._pick_vehicle()

        sdf_xml = (self._build_car_sdf(color_rgb)
                   if vtype == 'car' else self.ambulance_sdf)
        label   = f'car({color_name})' if vtype == 'car' else '★AMBULANCE'
        path    = PATHS_FROM[direction]
        sp      = SPAWN_POINTS[direction]

        # Spawn in Gazebo
        req = SpawnEntity.Request()
        req.name            = name
        req.xml             = sdf_xml
        req.robot_namespace = f'/{name}'
        req.initial_pose.position.x = sp['x']
        req.initial_pose.position.y = sp['y']
        req.initial_pose.position.z = SPAWN_Z
        req.initial_pose.orientation.z = math.sin(sp['yaw'] / 2.0)
        req.initial_pose.orientation.w = math.cos(sp['yaw'] / 2.0)
        self.spawn_cli.call_async(req)

        # Mark emergency — broadcast immediately so green activates BEFORE ambulance moves
        if vtype == EMERGENCY_TYPE:
            self.active_emergency = name
            axis = 'NS' if direction in ('A', 'C') else 'EW'
            spawn_dist = 44.0
            em_msg = String()
            em_msg.data = f'EMERGENCY_APPROACH:{axis}:{name}:ambulance:5:{spawn_dist:.1f}'
            self.em_pub.publish(em_msg)
            self.get_logger().warn(
                f'[spawner] ★ AMBULANCE {name} dir={direction} axis={axis} — '
                f'green broadcast at spawn ({spawn_dist}m)'
            )

        # Record cooldown
        self.last_spawn_time[direction] = time.monotonic()

        # Launch controller
        cmd = [
            'ros2', 'run', 'smart_intersection', 'vehicle_controller.py',
            '--ros-args',
            '-r', f'__ns:=/{name}',
            '-p', f'vehicle_name:={name}',
            '-p', f'vehicle_type:={vtype}',
            '-p', f'path:={path}',
        ]
        proc = subprocess.Popen(cmd)

        self.active[name] = {
            'proc': proc, 'path': path, 'dir': direction,
            'type': vtype, 'color': color_name or 'white',
            'spawn_time': time.monotonic(),
        }
        self.get_logger().info(
            f'[spawner] {name} ({label}) dir={direction}  '
            f'active={len(self.active)}/{MAX_ACTIVE}'
        )

    # ── Cleanup ──────────────────────────────────────────────────────────────
    def cleanup(self):
        now = time.monotonic()
        gone = set()

        # 1. Vehicles that reported DONE
        for name in list(self.done_vehicles):
            if name in self.active and name not in self.deleting:
                gone.add(name)
            self.done_vehicles.discard(name)

        # 2. Vehicles whose controller process exited
        for name, info in list(self.active.items()):
            if name not in self.deleting and info['proc'].poll() is not None:
                gone.add(name)

        # 3. Vehicles that drove past road end (>50 m from center)
        for name, (vx, vy) in list(self.positions.items()):
            if name in self.active and name not in self.deleting:
                if math.hypot(vx, vy) > 50.0:
                    gone.add(name)

        # 4. Stale positions for vehicles no longer active
        for name in list(self.positions.keys()):
            if name not in self.active and name not in self.deleting:
                del self.positions[name]

        # 5. Force-clear stale deleting entries (Gazebo callback may never fire)
        for name in list(self.deleting.keys()):
            if now - self.deleting[name] > DELETE_TIMEOUT:
                del self.deleting[name]
                self.get_logger().warn(
                    f'[spawner] Force-cleared stale deleting entry: {name}'
                )

        # Process gone set
        for name in gone:
            self.deleting[name] = now

            # Clear ambulance tracking
            if self.active_emergency == name:
                self.active_emergency = None
                self.regular_spawn_counter = 0
                self.get_logger().warn(
                    f'[spawner] ★ AMBULANCE {name} cleared — '
                    f'next ambulance in {EMERGENCY_CHANCE} cars'
                )

            info = self.active.pop(name, None)
            if info and info['proc'].poll() is None:
                try:
                    info['proc'].terminate()
                except Exception:
                    pass

            req = DeleteEntity.Request()
            req.name = name
            future = self.delete_cli.call_async(req)
            future.add_done_callback(lambda f, n=name: self._on_deleted(n))

            self.positions.pop(name, None)

        if gone:
            self.get_logger().info(
                f'[spawner] Cleaned {len(gone)} vehicles  '
                f'remaining={len(self.active)}'
            )

    def _on_deleted(self, name):
        self.deleting.pop(name, None)

    def destroy_node(self):
        for info in self.active.values():
            if info['proc'].poll() is None:
                try:
                    info['proc'].terminate()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSpawner()
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except RuntimeError:
                pass  # ModelStates message too large — transient, safe to ignore
    except KeyboardInterrupt:
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
