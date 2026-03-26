#!/usr/bin/env python3
"""
test_spawner.py — Controlled test spawner for camera-based priority validation
===============================================================================
Spawns a fixed number of cars per direction to test the traffic priority system:
  A (North) = 5 cars
  B (East)  = 4 cars
  C (South) = 3 cars
  D (West)  = 2 cars

All cars use straight-through paths. They are spawned in a staggered line so
they queue up behind the stop line during the initial ALL_RED phase.

After spawning, this node monitors and logs the signal phases to verify
that the system correctly prioritises the busiest road first.
"""

import math
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

# How many cars per direction
TEST_COUNTS = {
    'A': 5,   # North — most traffic
    'B': 4,   # East
    'C': 3,   # South
    'D': 2,   # West  — least traffic
}

# Spawn points: first car near junction, others staggered back
# Cars are spaced 6m apart in a queue
SPAWN_CONFIGS = {
    'A': {'base_x': -2.0, 'base_y':  14.0, 'dy':  6.0, 'dx': 0.0, 'yaw': -math.pi/2, 'path': 'A_to_C'},
    'B': {'base_x':  14.0, 'base_y':  2.0, 'dy': 0.0, 'dx':  6.0, 'yaw':  math.pi,    'path': 'B_to_D'},
    'C': {'base_x':  2.0, 'base_y': -14.0, 'dy': -6.0, 'dx': 0.0, 'yaw':  math.pi/2,  'path': 'C_to_A'},
    'D': {'base_x': -14.0, 'base_y': -2.0, 'dy': 0.0, 'dx': -6.0, 'yaw':  0.0,        'path': 'D_to_B'},
}

SPAWN_Z = 0.15


class TestSpawner(Node):
    def __init__(self):
        super().__init__('test_spawner')

        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.get_logger().info('TestSpawner: waiting for Gazebo spawn service...')
        self.spawn_cli.wait_for_service(timeout_sec=60.0)
        self.get_logger().info('TestSpawner: service ready.')

        self.pkg_path = get_package_share_directory('smart_intersection')
        self.count = 0
        self.controllers = []
        self.spawn_done = False

        # Track signal phases to verify priority order
        self.phase_log = []
        self.create_subscription(String, '/traffic_phase', self.phase_cb, 10)

        # Spawn all test vehicles after a short delay
        self.create_timer(2.0, self.spawn_all_vehicles)

        # Log phase analysis periodically
        self.create_timer(5.0, self.log_analysis)

        self.get_logger().info(
            f'TestSpawner: will spawn A={TEST_COUNTS["A"]}, '
            f'B={TEST_COUNTS["B"]}, C={TEST_COUNTS["C"]}, D={TEST_COUNTS["D"]} cars'
        )

    def phase_cb(self, msg: String):
        phase = msg.data
        # Track GREEN phases
        if 'GREEN' in phase and (not self.phase_log or self.phase_log[-1] != phase):
            self.phase_log.append(phase)
            self.get_logger().info(f'  📶 PHASE CHANGE: {phase}')

    def spawn_all_vehicles(self):
        if self.spawn_done:
            return
        self.spawn_done = True

        sdf_path = os.path.join(self.pkg_path, 'models', 'car', 'model.sdf')
        with open(sdf_path, 'r') as f:
            sdf_xml = f.read()

        for direction, count in TEST_COUNTS.items():
            cfg = SPAWN_CONFIGS[direction]
            self.get_logger().info(
                f'  Spawning {count} cars from direction {direction}...'
            )

            for i in range(count):
                self.count += 1
                name = f'test_car_{self.count}'

                # Stagger cars in a queue behind the stop line
                x = cfg['base_x'] + cfg['dx'] * i
                y = cfg['base_y'] + cfg['dy'] * i

                req = SpawnEntity.Request()
                req.name = name
                req.xml = sdf_xml
                req.robot_namespace = f'/{name}'
                req.initial_pose.position.x = float(x)
                req.initial_pose.position.y = float(y)
                req.initial_pose.position.z = SPAWN_Z
                req.initial_pose.orientation.z = math.sin(cfg['yaw'] / 2.0)
                req.initial_pose.orientation.w = math.cos(cfg['yaw'] / 2.0)
                self.spawn_cli.call_async(req)

                # Launch vehicle controller
                cmd = [
                    'ros2', 'run', 'smart_intersection', 'vehicle_controller.py',
                    '--ros-args',
                    '-r', f'__ns:=/{name}',
                    '-p', f'vehicle_name:={name}',
                    '-p', f'vehicle_type:=car',
                    '-p', f'path:={cfg["path"]}',
                ]
                proc = subprocess.Popen(cmd)
                self.controllers.append(proc)

                self.get_logger().info(
                    f'    → {name} at ({x:.1f}, {y:.1f}) path={cfg["path"]}'
                )

        total = sum(TEST_COUNTS.values())
        self.get_logger().info(
            f'\n'
            f'  ╔══════════════════════════════════════════════╗\n'
            f'  ║  TEST SPAWNING COMPLETE: {total} cars              ║\n'
            f'  ║  A(North)=5  B(East)=4  C(South)=3  D(West)=2  ║\n'
            f'  ║                                              ║\n'
            f'  ║  Expected priority order:                    ║\n'
            f'  ║    1st GREEN → A (North) — 5 cars            ║\n'
            f'  ║    2nd GREEN → B (East)  — 4 cars            ║\n'
            f'  ║    3rd GREEN → C (South) — 3 cars            ║\n'
            f'  ║    4th GREEN → D (West)  — 2 cars            ║\n'
            f'  ╚══════════════════════════════════════════════╝'
        )

    def log_analysis(self):
        if not self.phase_log:
            return
        greens = [p for p in self.phase_log if 'GREEN' in p]
        if len(greens) >= 2:
            self.get_logger().info(
                f'\n'
                f'  ── Signal Priority Analysis ──\n'
                f'  GREEN phases so far (in order): {greens}\n'
                f'  Expected order: N(5) → B/E(4) → S(3) → W(2)'
            )

    def destroy_node(self):
        for proc in self.controllers:
            if proc.poll() is None:
                proc.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TestSpawner()
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
