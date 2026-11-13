#!/usr/bin/env python3
"""
simulation_logger.py — State logging node
===========================================
Subscribes to all intersection telemetry (model states, traffic phases, emergency calls)
and writes a comprehensive JSONL log file for debugging and visualization.
Also prints a beautiful summary table to the console every 1 second.
"""

import math
import os
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates

LOG_DIR = '/home/dell/smart_traffic_ws/src/smart_intersection/logs'
LOG_FILE = os.path.join(LOG_DIR, 'simulation_state.jsonl')


class SimulationLogger(Node):
    def __init__(self):
        super().__init__('simulation_logger')

        # Create log directory if it doesn't exist
        os.makedirs(LOG_DIR, exist_ok=True)
        # Clear/truncate old log file on startup
        with open(LOG_FILE, 'w') as f:
            f.write('')

        self.get_logger().info(f'Simulation Logger initialized. Writing JSONL logs to: {LOG_FILE}')

        # Telemetry storage
        self.traffic_phase = 'ALL_RED'
        self.active_emergency_vehicle = None
        self.active_emergency_axis = None
        self.active_emergency_dist = 0.0

        self.vehicle_registry = {}  # name -> {type, spawn_dir, path_id}
        self.vehicle_history = {}   # name -> {last_x, last_y, last_time, speed}
        self.model_states = None

        # Subscriptions
        self.create_subscription(String, '/traffic_phase', self.traffic_cb, 10)
        self.create_subscription(String, '/emergency_request', self.emergency_cb, 10)
        self.create_subscription(String, '/vehicle_registry', self.registry_cb, 10)
        self.create_subscription(String, '/vehicle_done', self.done_cb, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_cb, 10)

        # Logging Timers
        self.create_timer(0.2, self.log_step)          # 5Hz file logging
        self.create_timer(1.0, self.console_summary)   # 1Hz console display

    def traffic_cb(self, msg: String):
        self.traffic_phase = msg.data

    def emergency_cb(self, msg: String):
        parts = msg.data.split(':')
        if len(parts) >= 3:
            action = parts[0]
            if action in ('EMERGENCY', 'EMERGENCY_APPROACH', 'EMERGENCY_CLOSE'):
                self.active_emergency_axis = parts[1]
                self.active_emergency_vehicle = parts[2]
                if len(parts) >= 6:
                    try:
                        self.active_emergency_dist = float(parts[5])
                    except ValueError:
                        pass
            elif action == 'CLEAR':
                if self.active_emergency_vehicle == parts[2]:
                    self.active_emergency_vehicle = None
                    self.active_emergency_axis = None
                    self.active_emergency_dist = 0.0

    def registry_cb(self, msg: String):
        parts = msg.data.split(':')
        if len(parts) >= 8:
            name, vtype, spawn_dir = parts[0], parts[1], parts[2]
            path_id = parts[7]
            self.vehicle_registry[name] = {
                'type': vtype,
                'spawn_dir': spawn_dir,
                'path_id': path_id
            }

    def done_cb(self, msg: String):
        name = msg.data
        self.vehicle_registry.pop(name, None)
        self.vehicle_history.pop(name, None)
        if self.active_emergency_vehicle == name:
            self.active_emergency_vehicle = None
            self.active_emergency_axis = None
            self.active_emergency_dist = 0.0

    def model_states_cb(self, msg: ModelStates):
        self.model_states = msg

    def _infer_signals(self):
        """Infers the signal color for N, S, E, W directions based on the current phase."""
        phase = self.traffic_phase
        signals = {'N': 'RED', 'S': 'RED', 'E': 'RED', 'W': 'RED'}

        if phase == 'ALL_RED':
            return signals

        if phase.startswith('PHASE:'):
            parts = phase.split(':')
            if len(parts) >= 3:
                phase_dir, phase_state = parts[1], parts[2]
                for direction in ('N', 'S', 'E', 'W'):
                    if direction in phase_dir:
                        signals[direction] = phase_state
        return signals

    def log_step(self):
        if self.model_states is None:
            return

        now = self.get_clock().now()
        timestamp = now.nanoseconds / 1e9

        signals = self._infer_signals()
        vehicles_data = []

        names = list(self.model_states.name)
        poses = self.model_states.pose

        for name in names:
            if not name.startswith('vehicle_'):
                continue

            try:
                idx = names.index(name)
                pose = poses[idx]
            except (ValueError, IndexError):
                continue

            x = pose.position.x
            y = pose.position.y
            dist = math.hypot(x, y)

            q = pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

            # Retrieve info from registry
            reg = self.vehicle_registry.get(name, {})
            vtype = reg.get('type', 'ambulance' if 'ambulance' in name else 'car')
            spawn_dir = reg.get('spawn_dir', 'A' if x > 0 else 'B')  # fallback

            # Calculate actual speed
            prev = self.vehicle_history.get(name)
            speed = 0.0
            if prev is not None:
                dt = timestamp - prev['last_time']
                if dt > 0.001:
                    dx = x - prev['last_x']
                    dy = y - prev['last_y']
                    speed = math.hypot(dx, dy) / dt
                    # Limit speed jump noise
                    speed = 0.8 * speed + 0.2 * prev['speed']

            self.vehicle_history[name] = {
                'last_x': x,
                'last_y': y,
                'last_time': timestamp,
                'speed': speed
            }

            # Map spawn_dir to light pole direction
            spawn_to_pole = {'A': 'N', 'B': 'E', 'C': 'S', 'D': 'W'}
            pole_dir = spawn_to_pole.get(spawn_dir[0], 'N')
            signal = signals.get(pole_dir, 'RED')

            vehicles_data.append({
                'name': name,
                'type': vtype,
                'x': round(x, 2),
                'y': round(y, 2),
                'yaw': round(yaw, 2),
                'dist_ctr': round(dist, 1),
                'speed': round(speed, 1),
                'signal': signal
            })

        log_record = {
            'timestamp': round(timestamp, 2),
            'traffic_phase': self.traffic_phase,
            'signals': signals,
            'active_emergency': {
                'vehicle': self.active_emergency_vehicle,
                'axis': self.active_emergency_axis,
                'dist': round(self.active_emergency_dist, 1) if self.active_emergency_vehicle else 0.0
            },
            'vehicles': vehicles_data
        }

        # Write to JSONL file
        try:
            with open(LOG_FILE, 'a') as f:
                f.write(json.dumps(log_record) + '\n')
        except IOError:
            pass

    def console_summary(self):
        if self.model_states is None:
            return

        signals = self._infer_signals()
        active_vehicles = [name for name in self.model_states.name if name.startswith('vehicle_')]

        print("\n" + "="*80)
        print(f" INTERSECTION STATE | Phase: {self.traffic_phase}")
        print(f" LIGHTS: N/S: {signals['N']} | E/W: {signals['E']}")
        if self.active_emergency_vehicle:
            print(f" 🚨 EMERGENCY: {self.active_emergency_vehicle} on axis {self.active_emergency_axis} (dist: {self.active_emergency_dist:.1f}m)")
        else:
            print(" 🚨 EMERGENCY: None")
        print("-"*80)

        if not active_vehicles:
            print(" No active vehicles in simulation.")
        else:
            print(f" {'VEHICLE':<12} | {'TYPE':<10} | {'POSITION (X,Y)':<15} | {'DIST (m)':<8} | {'SPEED':<6} | {'SIGNAL':<6}")
            print("-"*80)
            for name in sorted(active_vehicles):
                history = self.vehicle_history.get(name, {})
                reg = self.vehicle_registry.get(name, {})
                vtype = reg.get('type', 'ambulance' if 'ambulance' in name else 'car')
                speed = history.get('speed', 0.0)

                # Get latest coordinate
                try:
                    names = list(self.model_states.name)
                    idx = names.index(name)
                    pose = self.model_states.pose[idx]
                    x, y = pose.position.x, pose.position.y
                    dist = math.hypot(x, y)
                except (ValueError, IndexError):
                    continue

                spawn_dir = reg.get('spawn_dir', 'A')
                spawn_to_pole = {'A': 'N', 'B': 'E', 'C': 'S', 'D': 'W'}
                pole_dir = spawn_to_pole.get(spawn_dir[0], 'N')
                signal = signals.get(pole_dir, 'RED')

                vtype_str = f"★ {vtype.upper()}" if vtype == 'ambulance' else vtype
                pos_str = f"{x:5.1f}, {y:5.1f}"
                print(f" {name:<12} | {vtype_str:<10} | {pos_str:<15} | {dist:7.1f} | {speed:4.1f} | {signal:<6}")
        print("="*80 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = SimulationLogger()
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.05)
            except (RuntimeError, TypeError, ValueError):
                pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
