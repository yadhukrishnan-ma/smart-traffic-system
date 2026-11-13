# Smart Traffic Intersection

Professional-grade 4-way smart intersection simulation for ROS 2 Humble + Gazebo 11.

## Features

| Feature | Description |
|---------|-------------|
| **2 Vehicle Types** | Car (10 random colors) and Ambulance (white with red cross) |
| **4-Direction Traffic** | Vehicles spawn from North, East, South, and West with straight/turn paths |
| **Pure Pursuit Control** | Waypoint-based lane following with heading-error correction |
| **Adaptive Timing** | Green time = `base_time + (waiting_vehicles × density_bonus)`, max 25 s |
| **Camera-Based Counting** | Overhead cameras on each pole feed OpenCV background-subtraction counter |
| **Emergency Override** | Dual-detection (topic + camera) triggers instant green for ambulance lane |
| **Collision Prevention** | Narrow forward-cone check (18°) with distance-based slowdown/stop |
| **Self-Healing Nodes** | Orphan vehicle controllers self-terminate automatically when simulation stops |
| **State Logging** | Built-in logger node writes state records (JSONL) and prints real-time terminal tables |

## Priority Weights

| Type | Weight | Emergency |
|------|--------|-----------|
| Ambulance | 5 | ✅ |
| Car | 0 | ❌ |

## Spawn Distribution

- **Car:** Spawns 10 different colors sequentially.
- **Ambulance:** Spawns every **4 cars** (if no emergency is currently active).

Max active vehicles: 20. Spawn interval: 1.5 seconds. Spawns are round-robin across all 4 directions.

## Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/traffic_phase` | String | `PHASE:DIR:GREEN` / `PHASE:DIR:YELLOW` / `ALL_RED` |
| `/signal_timers` | String | `N:08,E:00,S:00,W:00` — remaining green seconds per direction |
| `/vehicle_registry` | String | Each vehicle reports: `name:type:dir:x:y:speed:dist:path` |
| `/vehicle_positions` | String | Controller broadcasts all positions for collision avoidance |
| `/vehicle_counts` | String | Camera-based count per direction: `DIR:COUNT` |
| `/vehicle_done` | String | Vehicle signals path completion for cleanup |
| `/emergency_request` | String | `EMERGENCY_APPROACH:AXIS:name:type:pw:dist` or `CLEAR:AXIS:name` |
| `/cam_*/image_raw` | Image | 4 overhead camera feeds (north, east, south, west) |

## Project Structure

```
smart_intersection/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── intersection.launch.py
├── worlds/
│   └── intersection.world
├── config/
│   └── waypoints.yaml
├── models/
│   ├── car/             (customizable color sedan)
│   ├── ambulance/       (white + red cross)
│   └── traffic_light/
└── scripts/
    ├── smart_traffic_controller.py   (signal cycling, lamp control, density, emergency)
    ├── vehicle_controller.py         (waypoint following, signal stop, collision avoid)
    ├── vehicle_spawner.py            (4-direction spawn, cleanup)
    ├── vehicle_counter.py            (OpenCV background subtraction on camera feeds)
    ├── emergency_detector.py         (dual topic+camera emergency detection)
    └── simulation_logger.py          (real-time state logger and terminal visualizer)
```

## Build & Run

```bash
cd ~/smart_traffic_ws
chmod +x src/smart_intersection/scripts/*.py
colcon build --packages-select smart_intersection
source install/setup.bash
ros2 launch smart_intersection intersection.launch.py
```

## Simulation & Priority Flow

1. **Simulation Startup:**
   - Gazebo launches the intersection world containing lane markings, traffic poles, and overhead camera sensors.
   - The **smart traffic controller** spawns color lamp spheres, starting the normal light cycling.
   - The **emergency detector** and **vehicle counter** begin monitoring the environment.

2. **Normal Cycle & Traffic Spawning:**
   - The **vehicle spawner** spawns cars every 1.5 seconds, choosing colors round-robin.
   - Cars follow waypoint paths using a **Pure Pursuit controller**.
   - Signals cycle automatically (Green -> Yellow -> Red). Vehicles stop before the stop line (8.0m) on Red, and accelerate to clear the junction if Yellow occurs when they are close (9.2m).
   - If cars queue up, **Adaptive Timing** extends green phases dynamically based on queue density.

3. **Ambulance Spawning & Immediate green:**
   - Every 4th vehicle spawned is an **Ambulance**.
   - The millisecond the ambulance is spawned, `vehicle_spawner.py` broadcasts `EMERGENCY_APPROACH` to `/emergency_request`.
   - The traffic controller receives this and instantly switches the ambulance lane to **GREEN** (reserving yellow/red only for the cross-axis).

4. **Yielding & Clearing the Path:**
   - To prevent blocking the ambulance, all normal vehicles in front of the ambulance accelerate to **3.3 m/s** (1.5x cruise speed) to clear the lane immediately.
   - The ambulance drives fast at **3.96 m/s** (1.8x speed) and ignores all traffic lights.
   - Because the collision cone is narrowed to **18°**, the ambulance never false-stops for vehicles traveling in opposite or adjacent lanes.

5. **Clearing the Junction:**
   - Once the ambulance crosses the center and exits the intersection, a `CLEAR` request is fired.
   - The traffic controller resets back to the normal cycle, clearing the emergency override.
   - The ambulance and other vehicles despawn automatically at the end of their paths.
   - Any background child controller processes automatically terminate if Gazebo stops publishing positions, preventing orphan processes.

## Viva Explanation Points

- **Pure Pursuit**: Each vehicle computes its steering angle using heading error relative to waypoints: `ω = k_steer * error`.
- **Narrowed Collision Cone (18°)**: Ensures collision avoidance only checks vehicles directly ahead in the same lane. This eliminates false-stops from traffic in the opposite direction.
- **Immediate Emergency Override**: Turns the target lane green at the exact moment of spawn, ensuring the ambulance never has to wait.
- **Orphan Auto-Termination**: Prevents background Python nodes from hanging across simulation runs.
