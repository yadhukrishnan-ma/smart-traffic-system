# Smart Traffic Intersection

Professional-grade 4-way smart intersection simulation for ROS 2 Humble + Gazebo 11.

## Features

| Feature | Description |
|---------|-------------|
| **5 Vehicle Types** | Car (blue), Bus (orange), Ambulance (white+cross), Fire Engine (red), Police (dark blue) |
| **4-Direction Traffic** | Vehicles spawn from North, East, South, and West with straight/turn paths |
| **Pure Pursuit Control** | Waypoint-based lane following with heading-error correction |
| **Adaptive Timing** | Green time = `base_time + (waiting_vehicles × density_bonus)`, max 25 s |
| **Camera-Based Counting** | Overhead cameras on each pole feed OpenCV background-subtraction counter |
| **Emergency Override** | Dual-detection (topic + camera) triggers instant green; 30 s safety timeout |
| **Collision Prevention** | Forward-cone check (±70°) with distance-based slowdown/stop |
| **Auto Cleanup** | Vehicles deleted when they finish their path |

## Priority Weights

| Type | Weight | Emergency |
|------|--------|-----------|
| Ambulance | 5 | ✅ |
| Fire Engine | 4 | ✅ |
| Police | 3 | ✅ |
| Bus | 1 | ❌ |
| Car | 0 | ❌ |

## Spawn Distribution

- 70% Car
- 15% Bus
- 5% Ambulance
- 5% Fire Engine
- 5% Police

Max vehicles: 8. Spawn interval: ~8 seconds. Round-robin across 4 directions.

## Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/traffic_phase` | String | `PHASE:DIR:GREEN` / `PHASE:DIR:YELLOW` / `ALL_RED` |
| `/signal_timers` | String | `N:08,E:00,S:00,W:00` — remaining green seconds per direction |
| `/vehicle_registry` | String | Each vehicle reports: `name:type:dir:x:y:speed:dist:path` |
| `/vehicle_positions` | String | Controller broadcasts all positions for collision avoidance |
| `/vehicle_counts` | String | Camera-based count per direction: `DIR:COUNT` |
| `/vehicle_done` | String | Vehicle signals path completion for cleanup |
| `/emergency_request` | String | `EMERGENCY:AXIS:name:type:pw:dist` or `CLEAR:AXIS:name` |
| `/intersection_lock` | String | `LOCKED:vehicle_name` or `UNLOCKED` |
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
│   ├── car/          (blue sedan)
│   ├── bus/          (orange bus)
│   ├── ambulance/    (white + red cross)
│   ├── fire_engine/  (red truck)
│   ├── police/       (dark blue + lights)
│   └── traffic_light/
└── scripts/
    ├── smart_traffic_controller.py   (signal cycling, lamp control, density, emergency)
    ├── vehicle_controller.py         (waypoint following, signal stop, collision avoid)
    ├── vehicle_spawner.py            (4-direction spawn, cleanup)
    ├── vehicle_counter.py            (OpenCV background subtraction on camera feeds)
    └── emergency_detector.py         (dual topic+camera emergency detection)
```

## Build & Run

```bash
cd ~/smart_traffic_ws
colcon build --packages-select smart_intersection
source install/setup.bash
ros2 launch smart_intersection intersection.launch.py
```

## Simulation Flow

1. Gazebo launches with intersection world (roads, sidewalks, lane markings, traffic lights, cameras)
2. Smart traffic controller spawns lamp spheres and camera mounts, starts with 20 s ALL_RED
3. Emergency detector starts listening for registry + camera dual-detection
4. Vehicle spawner begins spawning every ~8 s, round-robin across 4 directions
5. Vehicle counter processes overhead camera feeds via background subtraction
6. Vehicles follow waypoint paths via pure pursuit with heading-error correction
7. Vehicles stop at red lights, proceed on green (with yellow point-of-no-return at ≤4 m)
8. Density merges registry counts with camera counts; busier directions get longer green
9. Emergency vehicle dual-detected → instant green for that direction (30 s max)
10. Emergency passes center & exits → CLEAR fired → normal cycle resumes after YELLOW
11. Vehicles finish path → auto-deleted from Gazebo

## Viva Explanation Points

- **Pure Pursuit**: Each vehicle computes `heading_error = atan2(dy,dx) - yaw` and applies angular velocity `ω = k × error` with exponential smoothing
- **Forward-Cone Following**: Only vehicles within ±70° forward cone affect speed — prevents rear vehicles from blocking front of queue
- **Adaptive Timing**: Green phase duration dynamically extends when more vehicles are waiting (`base + count × bonus`), capped at 25 s
- **Density Merging**: Controller takes `max(registry_count, camera_count)` per direction for robust density estimation
- **Emergency Protocol**: Dual detection (topic reports + camera HSV analysis) required; controller forces green; 30 s safety timeout; 5 s cooldown prevents rapid re-triggers
- **4-Direction Clockwise Cycle**: N → E → S → W with score-based next-direction selection (`density × 10 + wait_time × 0.1`)
