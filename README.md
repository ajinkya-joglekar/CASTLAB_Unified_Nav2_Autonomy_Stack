# CAST Nav2 Base Stack

This repository provides a profile-driven outdoor navigation baseline for ROS 2 Jazzy. It runs the same Nav2, GPS localization, waypoint, visualization, and optional semantic-traversability pipeline in Gazebo Harmonic simulation or against a real vehicle's ROS topics.

The stack currently supports two Ackermann vehicles (Polaris Ranger and Jeep), two outdoor simulation worlds (CAST and Baylands), RViz, Mapviz, staged GUI bringup, and configurable topic adaptation for hardware. Semantic segmentation, terrain-grid generation, and the Nav2 terrain costmap layer are retained as optional components. Scene-graph generation is intentionally not part of this base repository.

## What “base stack” means

The base is the reusable layer between a vehicle and Nav2. It owns:

- Vehicle and world profile registries.
- Gazebo vehicle spawning, robot-state publication, and ROS/Gazebo bridges.
- Dual-EKF and NavSat localization.
- Nav2 planning, control, behaviors, collision monitoring, and waypoint following.
- Canonical ROS topic names plus adapters for platform-specific topic names.
- RViz and georeferenced Mapviz visualization.
- GPS waypoint playback, interactive waypoint commands, and waypoint logging.
- Optional semantic segmentation and terrain-aware navigation.
- A Tkinter GUI that launches simulation and navigation as separate stages.

It does not own physical sensor drivers, vehicle drive-by-wire software, or a terrain scene graph. In hardware mode those platform-specific systems must already be publishing and subscribing to the topics selected in the GUI or launch arguments.

## Runtime architecture

The baseline data flow is:

```text
Gazebo or hardware
  ├─ wheel odometry ─┐
  ├─ IMU ────────────┼─> dual EKF + NavSat transform ─> /odometry/filtered
  ├─ GPS fix ────────┘                                  │
  ├─ laser/points ───────────────────────────────────────┤
  │                                                     v
  └─ RGB-D camera ─> semantic segmentation ─> terrain grid ─> Nav2 terrain layer
                                                        │
                                                        v
                                            planner + controller ─> /cmd_vel
```

Perception is opt-in. With perception disabled, Nav2 uses the normal obstacle sources in the selected baseline Nav2 YAML. With perception enabled, semantic and terrain topics are produced, but they influence planning only when a terrain-aware Nav2 YAML is selected.

### Geospatial localization

Gazebo worlds define a WGS84 spherical-coordinate origin. Gazebo's NavSat system converts the simulated vehicle position into `sensor_msgs/NavSatFix`; on hardware, a GPS driver supplies the equivalent message.

`robot_localization` provides three cooperating nodes:

1. The local EKF fuses odometry and IMU data, publishes `/odometry/local`, and maintains the local `odom` frame.
2. `navsat_transform_node` converts latitude/longitude into Cartesian `/odometry/gps` using GPS, heading, and filtered odometry.
3. The global EKF fuses local motion with GPS position, publishes `/odometry/filtered`, and maintains the global `map` relationship.

The important transform chain is:

```text
map -> odom -> base_link -> sensors
```

Nav2 plans in `map`, controls `base_link`, and consumes `/odometry/filtered`. Robot descriptions provide the fixed transforms from `base_link` to LiDAR, IMU, GPS, and camera frames.

### Navigation

The vehicle profiles use Ackermann-oriented Nav2 configurations. The stack includes the planner, controller, smoother, behavior server, BT navigator, waypoint follower, velocity smoother, collision monitor, and lifecycle management. Baseline and terrain-aware YAML files are separate so terrain integration is always an explicit choice.

### Semantic traversability

The optional perception chain contains three packages:

- `semantic_segmentation_node`: runs an ONNX segmentation model and publishes the class mask, confidence, labels, and overlay.
- `terrain_obstacle_mapper`: combines the segmentation output with organized RGB-D points and publishes obstacle and semantic grids.
- `terrain_costmap_layer`: imports the terrain grid into a Nav2 costmap.

Canonical outputs include:

- `/segmentation/mask`
- `/segmentation/confidence`
- `/segmentation/label_info`
- `/segmentation/overlay`
- `/terrain/obstacle_grid`
- `/terrain/semantic_grid`

The canonical terrain classes are `background_unknown`, `hard_traversable`, `grass`, `mud`, `water`, `vegetation`, and `obstacle`. Model profiles and any class conversion are defined in `ws/src/base_nav2_stack/config/perception/segmentation_models.yaml`.

To make terrain affect Nav2, enable perception and select one of:

- `config/nav2/ranger_terrain.yaml`
- `config/nav2/jeep_terrain.yaml`

The ordinary `ranger.yaml` and `jeep.yaml` configurations do not load the terrain layer.

## Repository layout

```text
.
├── Dockerfile
├── .bash_alias_container
├── README.md
└── ws
    ├── .bashrc_nav2
    └── src
        ├── base_nav2_stack
        ├── semantic_segmentation_node
        ├── terrain_obstacle_mapper
        └── terrain_costmap_layer
```

`base_nav2_stack` contains the launch files, GUI, vehicle/world registries, localization and Nav2 parameters, URDFs, meshes, world assets, RViz/Mapviz configurations, and waypoint utilities.

## Prerequisites

- Linux host with Docker.
- An X11 display available to Docker.
- NVIDIA Container Toolkit and a compatible GPU/driver for GPU passthrough.
- A Stadia Maps API key if satellite tiles are needed in Mapviz.

The image is based on ROS 2 Jazzy Desktop for Ubuntu Noble and installs Gazebo Harmonic, Nav2, `robot_localization`, ROS/Gazebo bridges, Mapviz, RGB-D processing, and ONNX Runtime.

## Build the Docker image

From the repository root:

```bash
docker build --network=host --no-cache --pull -t nav2-base-jazzy .
```

Or load the host aliases and build:

```bash
source .bash_alias_container
build_base
```

The image build compiles exactly these workspace packages:

```text
base_nav2_stack
semantic_segmentation_node
terrain_obstacle_mapper
terrain_costmap_layer
```

## Run the development container

The provided container alias runs with host networking, X11, GPU access, the host user ID, and a live mount of `ws` at `/ws`. First create the identity files used by that alias if they do not already exist:

```bash
mkdir -p .docker_identity
cp /etc/passwd .docker_identity/passwd
cp /etc/group .docker_identity/group
```

Then start a new persistent container:

```bash
source .bash_alias_container
nav2_base_run
```

Useful host aliases are:

| Alias | Purpose |
|---|---|
| `build_base` | Rebuild the Docker image as `nav2-base-jazzy`. |
| `nav2_base_run` | Create and enter the persistent `nav2-base-dev` container. |
| `nav2_base_start` | Re-enter the stopped container. |
| `nav2_base_exec` | Open another shell in a running container. |
| `nav2_base_rm` | Remove the stopped development container. |

Because the entire workspace is mounted, host edits are immediately visible inside the container. Rebuild affected ROS packages before launching them.

## Build inside the container

The container shell loads `/ws/.bashrc_nav2`, sources ROS, and sources `/ws/install/setup.bash` when available.

```bash
# Build every retained package
build

# Build only the base package
buildbase

# Clean and rebuild all four custom packages
rebuildcustom

# Clean and rebuild the three perception/terrain packages
rebuildterrain
```

Individual helpers include `buildss`, `buildtm`, `buildtcost`, `rebuildbase`, `rebuildss`, `rebuildtm`, and `rebuildtcost`.

## Launch with the GUI

Inside the container, run:

```bash
navgui
```

This is an alias for:

```bash
ros2 run base_nav2_stack launch_gui
```

The GUI deliberately separates simulation from navigation. This makes startup order visible and lets you inspect or correct topic mappings before Nav2 begins.

### Recommended simulation workflow

1. Select `Sim`, a vehicle, and a compatible world.
2. Leave localization as `managed` for the built-in dual-EKF pipeline.
3. Confirm the spawn pose and YAML selections.
4. Choose RViz and/or Mapviz.
5. If terrain navigation is desired, enable perception and select the matching `*_terrain.yaml` Nav2 configuration.
6. Click **Launch Sim**.
7. Wait for Gazebo to load and the vehicle to spawn.
8. Click **Refresh Topics**, inspect **Map Topics**, and correct any unexpected mappings.
9. Click **Launch Nav2**.
10. Use **Show Logs** to inspect both child processes. Use **Stop Nav2**, **Stop Sim**, or **Stop All** for controlled shutdown.

### GUI controls

| Control | Meaning |
|---|---|
| Mode | `Sim` starts Gazebo and bridges; `Hardware` assumes external platform nodes are already running. |
| Vehicle | Loads the vehicle's URDF, Nav2/EKF defaults, RViz configuration, compatible worlds, and default topics from `vehicles.yaml`. |
| World | Loads an SDF, Gazebo world name, and spawn defaults from `worlds.yaml`. Available only in simulation. |
| Gazebo world name | Namespace Gazebo uses for world-scoped topics such as `/world/<name>/clock`. Normally populated by the world profile. |
| Localization | `managed` starts the dual-EKF/NavSat pipeline; `external` adapts an existing filtered-odometry topic instead. |
| Spawn x/y/z/yaw | Simulation spawn pose. Changing the world restores that world's registered defaults. |
| EKF YAML | Optional override for the selected vehicle's localization configuration. |
| Nav2 YAML | Optional override for the selected vehicle's Nav2 configuration. This is where baseline versus terrain-aware navigation is chosen. |
| Enable perception/traversability | Starts segmentation, RGB-D point generation when selected, and terrain mapping. |
| Segmentation model | Chooses a registered ONNX/ontology profile. |
| Custom ONNX model | Overrides the registered model file. |
| Custom ontology YAML | Overrides the registered class/ontology file. |
| Terrain mapper YAML | Selects mapper geometry, thresholds, and class behavior. |
| Semantic profile override | Optional mapper-side semantic conversion profile. |
| Generate RGB-D points | Builds an organized point cloud from depth image and camera info. Disable only if the platform already publishes a trusted organized cloud. |
| RViz / Mapviz | Selects visualization processes launched with Nav2. |
| Composition | Runs supported Nav2 nodes in a component container. |
| Autostart | Automatically activates Nav2 lifecycle nodes. |
| Respawn | Requests process respawn for supported Nav2 nodes. |

The command preview shows the exact `ros2 launch` commands the GUI will execute for the simulation and navigation stages.

### Topic mapping panel

The GUI maintains canonical internal names while allowing each platform to use its native names. **Refresh Topics** runs `ros2 topic list -t`; **Map Topics** presents discovered topics grouped by message type.

Mappings cover command velocity, raw and filtered odometry, IMU, GPS, laser scan, point cloud, and RGB-D camera topics. In hardware mode, lightweight relay nodes are created only when the platform topic differs from the canonical name.

Changing a source mapping after a stage starts requires restarting Nav2. If the mapping controls a Gazebo bridge, restart simulation as well. The GUI displays this warning rather than silently leaving the old mapping active.

### Simulation and hardware behavior

`Launch Sim` executes `launch_stage:=sim`, which starts Gazebo, the selected robot, robot state publisher, and bridges. `Launch Nav2` executes `launch_stage:=nav`, which starts perception when enabled, localization when managed, Nav2, and visualization.

In hardware mode:

- Gazebo and ROS/Gazebo bridges are not started.
- Sensor and vehicle drivers are not started.
- Managed localization expects raw odometry, IMU, and GPS.
- External localization expects an already-filtered odometry source.
- The stack adapts selected platform topics to its canonical internal interface.

## Command-line launches

The GUI is optional. A complete Ranger simulation can be launched directly:

```bash
ros2 launch base_nav2_stack base_stack.launch.py \
  mode:=sim vehicle:=ranger world:=CAST_new use_rviz:=true
```

The `basenav` container alias runs this command.

To launch the two stages separately:

```bash
ros2 launch base_nav2_stack base_stack.launch.py \
  launch_stage:=sim mode:=sim vehicle:=ranger world:=CAST_new

ros2 launch base_nav2_stack base_stack.launch.py \
  launch_stage:=nav mode:=sim vehicle:=ranger localization_mode:=managed
```

Terrain-aware navigation requires both the perception flag and terrain Nav2 YAML:

```bash
ros2 launch base_nav2_stack base_stack.launch.py \
  mode:=sim vehicle:=ranger world:=CAST_new \
  use_perception:=true segmentation_model:=rellis_7_class \
  nav2_config:=config/nav2/ranger_terrain.yaml
```

Managed hardware localization example:

```bash
ros2 launch base_nav2_stack base_stack.launch.py \
  mode:=hardware vehicle:=ranger localization_mode:=managed \
  odom_topic:=/vehicle/odom \
  imu_topic:=/vehicle/imu \
  gps_topic:=/vehicle/gps/fix \
  points_topic:=/ouster/points \
  cmd_vel_topic:=/vehicle/cmd_vel
```

External hardware localization example:

```bash
ros2 launch base_nav2_stack base_stack.launch.py \
  mode:=hardware vehicle:=ranger localization_mode:=external \
  filtered_odom_topic:=/vehicle/odometry/filtered
```

## Mapviz satellite imagery

The checked-in Mapviz configuration contains the placeholder `api_key=api_key`; no credential is stored in the repository. After building, inject the current shell's key into the installed configuration:

```bash
export STADIA_API_KEY='your-key'
sed -i "s|api_key=api_key|api_key=${STADIA_API_KEY}|g" \
  /ws/install/base_nav2_stack/share/base_nav2_stack/config/gps_wpf_castlab.mvc
```

Then enable Mapviz in the GUI or launch with `use_mapviz:=true`. Rebuilding or removing the package install may restore the placeholder.

## GPS waypoint utilities

Three helper executables are installed with the base package:

```bash
ros2 run base_nav2_stack logged_waypoint_follower
ros2 run base_nav2_stack interactive_waypoint_follower
ros2 run base_nav2_stack gps_waypoint_logger
```

`logged_waypoint_follower` sends the geodetic waypoints in `config/demo_waypoints.yaml`. `interactive_waypoint_follower` converts Mapviz clicks into geographic goals. `gps_waypoint_logger` provides a Tkinter interface for capturing GPS position and orientation into a reusable waypoint YAML file.

## Extending the registries

Vehicle profiles live in `ws/src/base_nav2_stack/config/vehicles.yaml`. A vehicle entry identifies its URDF, default Nav2 and EKF configurations, RViz configuration, compatible worlds, and native topic names.

World profiles live in `ws/src/base_nav2_stack/config/worlds.yaml`. A world entry identifies its SDF, Gazebo world name, and default spawn pose.

The base launch and GUI validate both registries at startup. Add assets under `base_nav2_stack`, register them with package-relative paths, rebuild the package, and confirm the new profile appears in the GUI.

## Validation and troubleshooting

After a build, first check package discovery:

```bash
ros2 pkg prefix base_nav2_stack
ros2 pkg prefix semantic_segmentation_node
ros2 pkg prefix terrain_obstacle_mapper
ros2 pkg prefix terrain_costmap_layer
```

After launching simulation and Nav2, inspect:

```bash
ros2 node list
ros2 topic list -t
ros2 lifecycle nodes
ros2 action list
ros2 run tf2_tools view_frames
```

Common failure modes:

- **GUI does not open:** verify X11 forwarding, `DISPLAY`, and that the container can access `/tmp/.X11-unix`.
- **No Gazebo clock:** confirm the GUI's Gazebo world name matches the actual `/world/<name>/clock` namespace.
- **Nav2 remains inactive:** inspect lifecycle nodes and leave Autostart enabled unless manual lifecycle control is intended.
- **No global localization:** verify GPS, IMU, odometry, timestamps, frame IDs, and the `map -> odom -> base_link` chain.
- **Terrain topics exist but navigation ignores them:** select the matching `*_terrain.yaml`; enabling perception alone does not load the costmap plugin.
- **Terrain projection is distorted:** ensure the depth point cloud is organized and pixel-aligned with the segmentation mask.
- **Hardware commands go to the wrong topic:** update command-velocity mapping and restart Nav2.
- **Old ROS graph entries remain:** run `nav2_clean`, then inspect `nodes` and `topics` before relaunching.
