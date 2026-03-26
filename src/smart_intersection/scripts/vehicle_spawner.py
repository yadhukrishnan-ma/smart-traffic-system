#!/usr/bin/env python3
"""
vehicle_spawner.py — Full 4-direction vehicle spawner
=====================================================
Spawns vehicles from all 4 approach directions (A=North, B=East, C=South, D=West).
Supports multiple vehicle types: car, bus, ambulance, fire_engine, police.
Round-robins through directions, checks lane clearance before spawning.
"""

import math
import os
import random
import subprocess

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

# Spawn points: direction letter → (x, y, yaw)
# Vehicle faces toward junction center
SPAWN_POINTS = {
    'A': {'x':  2.0, 'y':  45.0, 'yaw': -math.pi / 2},   # North → faces south
    'B': {'x': 45.0, 'y': -2.0, 'yaw':  math.pi},        # East  → faces west
    'C': {'x': -2.0, 'y': -45.0, 'yaw':  math.pi / 2},    # South → faces north
    'D': {'x':-45.0, 'y':  2.0, 'yaw':  0.0},             # West  → faces east
}

# Available paths from each direction (Straight only)
PATHS_FROM = {
    'A': ['A_to_C'],
    'B': ['B_to_D'],
    'C': ['C_to_A'],
    'D': ['D_to_B'],
}
# 100% straight
PATH_WEIGHTS = [100]

# Vehicle type selection
VEHICLE_TYPES = [
    ('car',         55),
    ('bus',         12),
    ('truck',        8),
    ('ambulance',    5),
    ('fire_engine',  5),
    ('police',       5),
]

MAX_ACTIVE     = 12      # max vehicles in sim at once
SPAWN_INTERVAL = 5.0     # seconds between spawn attempts
SPAWN_Z        = 0.15    # wheel bottom clearance
LANE_CLEAR_DIST = 20.0   # min distance from spawn point to allow new spawn


class VehicleSpawner(Node):
    def __init__(self):
        super().__init__('vehicle_spawner')

        self.spawn_cli  = self.create_client(SpawnEntity,  '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')

        self.get_logger().info('VehicleSpawner: waiting for Gazebo spawn service...')
        self.spawn_cli.wait_for_service(timeout_sec=30.0)
        self.get_logger().info('VehicleSpawner: ready.')

        self.pkg_path = get_package_share_directory('smart_intersection')

        self.count         = 0
        self.active        = {}    # name → {proc, path, dir, type}
        self.positions     = {}    # name → (x, y)
        self.done_vehicles = set()
        self.deleting      = set()

        # Round-robin direction index
        self.dir_order = ['A', 'B', 'C', 'D']
        self.dir_index = 0

        self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_states_cb, 10
        )
        self.create_subscription(String, '/vehicle_done', self.done_cb, 10)

        self.create_timer(SPAWN_INTERVAL, self.try_spawn)
        self.create_timer(2.0,            self.cleanup)

        self.get_logger().info(
            f'VehicleSpawner: 4 directions, every {SPAWN_INTERVAL}s, '
            f'max {MAX_ACTIVE} active'
        )

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
        self.get_logger().info(f'[spawner] {msg.data} reported DONE — will delete')

    def _lane_clear(self, direction):
        """Check no vehicle is within LANE_CLEAR_DIST of spawn point."""
        sp = SPAWN_POINTS[direction]
        sx, sy = sp['x'], sp['y']
        for name, (vx, vy) in self.positions.items():
            if math.hypot(vx - sx, vy - sy) < LANE_CLEAR_DIST:
                return False
        return True

    def _pick_vehicle_type(self):
        """Weighted random vehicle type selection."""
        types, weights = zip(*VEHICLE_TYPES)
        return random.choices(types, weights=weights, k=1)[0]

    def try_spawn(self):
        if len(self.active) >= MAX_ACTIVE:
            return

        # Try each direction in round-robin; skip if lane blocked
        for _ in range(4):
            direction = self.dir_order[self.dir_index]
            self.dir_index = (self.dir_index + 1) % 4

            if self._lane_clear(direction):
                self._do_spawn(direction)
                return

        self.get_logger().info('[spawner] All lanes occupied — skipping')

    def _do_spawn(self, direction):
        self.count += 1
        name  = f'vehicle_{self.count}'
        vtype = self._pick_vehicle_type()
        path  = random.choices(PATHS_FROM[direction], weights=PATH_WEIGHTS, k=1)[0]

        sdf_path = os.path.join(self.pkg_path, 'models', vtype, 'model.sdf')
        if not os.path.exists(sdf_path):
            # Fall back to car if model doesn't exist
            sdf_path = os.path.join(self.pkg_path, 'models', 'car', 'model.sdf')
            vtype = 'car'

        with open(sdf_path, 'r') as f:
            sdf_xml = f.read()

        sp = SPAWN_POINTS[direction]

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

        # Launch controller subprocess
        cmd = [
            'ros2', 'run', 'smart_intersection', 'vehicle_controller.py',
            '--ros-args',
            '-r', f'__ns:=/{name}',
            '-p', f'vehicle_name:={name}',
            '-p', f'vehicle_type:={vtype}',
            '-p', f'path:={path}',
        ]
        proc = subprocess.Popen(cmd)

        self.active[name] = {'proc': proc, 'path': path, 'dir': direction, 'type': vtype}
        self.get_logger().info(
            f'[spawner] Spawned {name} ({vtype}) dir={direction} path={path}  '
            f'active={len(self.active)}/{MAX_ACTIVE}'
        )

    def cleanup(self):
        gone = set()

        for name in list(self.done_vehicles):
            if name in self.active and name not in self.deleting:
                gone.add(name)
            self.done_vehicles.discard(name)

        for name, info in list(self.active.items()):
            if name not in self.deleting and info['proc'].poll() is not None:
                gone.add(name)

        for name in gone:
            self.deleting.add(name)

            info = self.active.pop(name, None)
            if info and info['proc'].poll() is None:
                info['proc'].terminate()

            req = DeleteEntity.Request()
            req.name = name
            future = self.delete_cli.call_async(req)
            future.add_done_callback(lambda f, n=name: self._on_deleted(n))

            self.positions.pop(name, None)
            self.get_logger().info(f'[spawner] Deleting {name}')

    def _on_deleted(self, name):
        self.deleting.discard(name)
        self.get_logger().info(f'[spawner] Gazebo confirmed deletion of {name}')

    def destroy_node(self):
        for info in self.active.values():
            if info['proc'].poll() is None:
                info['proc'].terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSpawner()
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
