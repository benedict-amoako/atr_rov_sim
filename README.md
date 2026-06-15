# ATR ROV — ROS 2 Simulation Stack

> **Phase 2 of the ATR ROV Control System Development**
> ROS 2 Humble · Ignition Fortress · ros_gz Bridge · RViz2

This repository contains the ROS 2 simulation environment for the ATR multipurpose ROV. It provides a full physics-based simulation using Ignition Fortress (Gazebo), a ros_gz bridge for bidirectional topic translation, and RViz2 visualisation. The simulation is the primary development and validation environment for the ROV control system prior to deployment on physical hardware.

---

## Repository Structure

```
ros2_ws/
└── src/
    ├── atr_rov_description/        # Robot description package
    │   ├── urdf/
    │   │   └── atr_rov.urdf.xacro  # Full robot model — mass, inertia, sensors, thruster frames, Gazebo plugins
    │   ├── meshes/
    │   │   └── 4mm_houston_scaled_32mb.stl  # Hull visual mesh
    │   ├── launch/
    │   │   └── display_launch.py   # RViz2 display only (no Gazebo)
    │   ├── config/
    │   │   └── rviz_config.rviz    # Saved RViz2 configuration
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── atr_rov_gazebo/             # Gazebo simulation package
        ├── worlds/
        │   └── underwater.sdf      # Ignition world — physics, buoyancy, lighting, ocean floor
        ├── launch/
        │   └── gazebo_launch.py    # Full simulation launch — Gazebo + ROV + bridge + RViz2
        ├── config/
        │   └── ros_gz_bridge.yaml  # Topic bridge mapping — 8 thruster topics + odometry + IMU + clock
        ├── CMakeLists.txt
        └── package.xml
```

---

## System Requirements

| Requirement | Version |
|---|---|
| Operating System | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo | Ignition Fortress (version 6.x) |
| Python | 3.10 |

> **Note:** Do not use Snap packages for applications that run in the same desktop session as ROS 2. Snap runtimes inject library paths that conflict with ROS 2's vendored libraries (notably `libpthread` and `libOgreMain`). If RViz2 fails with a `symbol lookup error` referencing a `/snap/core20/` path, see the Troubleshooting section.

---

## Installation

### 1. System Prerequisites

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl gnupg lsb-release build-essential
sudo apt install -y python3-pip python3-colcon-common-extensions
sudo apt install -y python3-rosdep python3-vcstool
```

### 2. ROS 2 Humble

```bash
# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  https://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop

# Source ROS 2 in every new terminal
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify:
```bash
ros2 --version
# Expected: ros2 cli version X.X.X (from Python 3.10.X)
```

### 3. Ignition Fortress

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt install -y ignition-fortress
```

Verify:
```bash
ign gazebo --version
# Expected: Gazebo Sim, version 6.X.X
```

### 4. ros_gz Bridge and Simulation Packages

```bash
sudo apt-get install ros-humble-ros-gz
sudo apt install -y ros-humble-ros-gz-sim
sudo apt install -y ros-humble-ros-gz-bridge
```

### 5. Additional ROS 2 Tools

```bash
sudo apt install -y ros-humble-xacro
sudo apt install -y ros-humble-robot-state-publisher
sudo apt install -y ros-humble-joint-state-publisher-gui
sudo apt install -y ros-humble-rviz2
```

### 6. Initialise rosdep

Run once per machine:

```bash
sudo rosdep init
rosdep update
```

---

## Building the Workspace

```bash
cd ~/ros2_ws
colcon build --packages-select atr_rov_description atr_rov_gazebo
source install/setup.bash
```

> Always run `source install/setup.bash` after every build and in every new terminal that needs to use these packages.

---

## Running the Simulation

### RViz2 Display Only (no Gazebo)

Use this to verify the robot model, mesh, and TF frames without starting the physics simulation:

```bash
ros2 launch atr_rov_description display_launch.py
```

Expected result: RViz2 opens with the ROV mesh loaded and all TF frames visible — `base_link`, 8 thruster frames, 3 sensor frames, and the sonar frame.

### Full Gazebo Simulation

```bash
# Terminal 1 — launch everything
ros2 launch atr_rov_gazebo gazebo_launch.py

# Terminal 2 — monitor odometry
source ~/ros2_ws/install/setup.bash
ros2 topic echo /nav/ins_odom

# Terminal 3 — verify all topics are present
source ~/ros2_ws/install/setup.bash
ros2 topic list
```

### Manual Thruster Commands

Send open-loop force commands directly to individual thrusters for validation testing:

```bash
# Heave — fire all 4 vertical thrusters at 12.5 N each (50 N total)
ros2 topic pub --once /atr_mp_rov/thrusters/1/force std_msgs/msg/Float64 "{data: 12.5}"
ros2 topic pub --once /atr_mp_rov/thrusters/2/force std_msgs/msg/Float64 "{data: 12.5}"
ros2 topic pub --once /atr_mp_rov/thrusters/3/force std_msgs/msg/Float64 "{data: 12.5}"
ros2 topic pub --once /atr_mp_rov/thrusters/4/force std_msgs/msg/Float64 "{data: 12.5}"

# Surge — fire both horizontal thrusters at 20 N each
ros2 topic pub --once /atr_mp_rov/thrusters/5/force std_msgs/msg/Float64 "{data: 20.0}"
ros2 topic pub --once /atr_mp_rov/thrusters/6/force std_msgs/msg/Float64 "{data: 20.0}"
```

---

## Topic Reference

| ROS 2 Topic | Direction | Type | Description |
|---|---|---|---|
| `/atr_mp_rov/thrusters/1/force` | ROS 2 → Gazebo | `std_msgs/Float64` | T1 vertical front-left force command (N) |
| `/atr_mp_rov/thrusters/2/force` | ROS 2 → Gazebo | `std_msgs/Float64` | T2 vertical front-right force command (N) |
| `/atr_mp_rov/thrusters/3/force` | ROS 2 → Gazebo | `std_msgs/Float64` | T3 vertical rear-left force command (N) |
| `/atr_mp_rov/thrusters/4/force` | ROS 2 → Gazebo | `std_msgs/Float64` | T4 vertical rear-right force command (N) |
| `/atr_mp_rov/thrusters/5/force` | ROS 2 → Gazebo | `std_msgs/Float64` | T5 surge port force command (N) |
| `/atr_mp_rov/thrusters/6/force` | ROS 2 → Gazebo | `std_msgs/Float64` | T6 surge starboard force command (N) |
| `/atr_mp_rov/thrusters/7/force` | ROS 2 → Gazebo | `std_msgs/Float64` | T7 sway aft force command (N) |
| `/atr_mp_rov/thrusters/8/force` | ROS 2 → Gazebo | `std_msgs/Float64` | T8 sway forward force command (N) |
| `/nav/ins_odom` | Gazebo → ROS 2 | `nav_msgs/Odometry` | Vehicle pose and velocity at 100 Hz |
| `/ins/imu` | Gazebo → ROS 2 | `sensor_msgs/Imu` | IMU angular velocity and linear acceleration at 100 Hz |
| `/clock` | Gazebo → ROS 2 | `rosgraph_msgs/Clock` | Simulation clock for use_sim_time |

---

## Coordinate Frames

The simulation uses **NED (North-East-Down)** convention consistent with the Phase 1 mathematical model and the Fossen hydrodynamic formulation:

| Axis | Direction |
|---|---|
| X | Forward (bow) |
| Y | Starboard (right when facing forward) |
| Z | Positive downward |

RViz2 uses ENU (East-North-Up, Z positive upward). The `display_launch.py` applies a corrective 180° roll on the `world_to_base` joint so the model appears upright in RViz2. This correction is absent in the Gazebo launch — Gazebo operates in NED throughout.

---

## Phase 1 Parameters Embedded in the Model

All values sourced from Phase 1 — Mathematical Modelling (MATLAB/Simulink), document version 2.0.

| Parameter | Value | Unit |
|---|---|---|
| Mass | 119.0 | kg |
| Ixx | 6.91 | kg·m² |
| Iyy | 37.73 | kg·m² |
| Izz | 36.20 | kg·m² |
| CoM (x, y, z) | 0.06136, -0.00281, -0.1292 | m |
| CoB (x, y, z) | -0.071, 0.003198, -0.163076 | m |
| Added mass X | -53.65 | kg |
| Added mass Y/Z | -160.95 | kg |
| Linear damping X | -40.43 | N·s/m |
| Linear damping Y | -171.37 | N·s/m |
| Linear damping Z | -348.87 | N·s/m |

---

## Validation Tests

Run after the full Gazebo simulation is stable. All five must pass before Phase 2 is complete.

| Test | Description | Pass Condition |
|---|---|---|
| V1 | RViz2 model display | Mesh visible, all 13 TF frames present, no errors |
| V2 | Static hover — neutral buoyancy | `\|dz/dt\|` < 0.05 m/s after 5 seconds with no thrust |
| V3 | Thruster bridge connectivity | All 8 topics in `ros2 topic list`; T1 command produces Z velocity |
| V4 | Heave response | Measured acceleration within 25% of predicted 0.179 m/s² |
| V5 | Sensor topic health | `/nav/ins_odom` and `/ins/imu` at > 95 Hz; no NaN values |

---

## Known Issues

| ID | Issue | Status |
|---|---|---|
| F-01 | Inertia ixz and iyz signs in xacro need verification against Phase 1 rov_params.m | Open |
| F-02 | Thruster rpy values for T5–T8 need physical validation in test V3 | Open |
| F-03 | Fluid density hardcoded as 1000 kg/m³ — incorrect for saltwater | Carry to Phase 3 |
| F-04 | OdometryPublisher does not populate twist covariance | Carry to Phase 3 |
| F-05 | Sonar Sensors plugin must be declared at world level in SDF, not in xacro | Open |
| F-06 | Mesh origin does not align with base_link origin — pending corrected STL from mechanical team | Open |
| F-07 | Rotational damping values (Kp, Mq, Nr) artificially raised — real values pending water trial | Carry to Phase 7 |

---

## Troubleshooting

**RViz2 crashes with `symbol lookup error: /snap/core20/current/lib/.../libpthread.so.0`**

This is a Snap runtime library conflict. Snap injects its own `libpthread` into processes at the kernel namespace level, overriding the system glibc that ROS 2 was compiled against. The only reliable fix is to remove the snap packages that depend on `core20`:

```bash
# Check what depends on core20
snap connections --all | grep core20

# Remove dependents and then core20
sudo snap remove <dependent-snap>
sudo snap remove core20 --purge
```

Common core20 dependents: `foxglove-studio` (replace with the deb from github.com/foxglove/studio/releases), `gnome-3-28-1804` (safe to remove on Ubuntu 22.04).

---

**RViz2 opens but shows `Package [atr_rov_description] does not exist`**

The workspace has not been sourced in the current terminal:

```bash
source ~/ros2_ws/install/setup.bash
```

---

**Changes to the xacro have no effect after relaunching**

The `install/` directory is what gets served at runtime, not `src/`. Always rebuild after editing:

```bash
colcon build --packages-select atr_rov_description
source install/setup.bash
```

---

**Ignition topics not visible in ROS 2**

The ros_gz bridge may not have started. Check Terminal 1 output for bridge errors, then verify Ignition topics directly:

```bash
ign topic -l
ign topic -l | grep thrusters
```

---

## Development Phases

| Phase | Description | Status |
|---|---|---|
| Phase 1 | Mathematical modelling — MATLAB/Simulink, EOM, TAM, WLS allocator | Complete |
| Phase 2 | Gazebo simulation environment — this repository | In Progress |
| Phase 3 | Depth and navigation sensor driver | Pending |
| Phase 4 | Control system implementation — cascaded PID, ROS 2 control nodes | Pending |
| Phase 5 | Hardware integration — Teensy 4.1, micro-ROS, thruster and sensor interfacing | Pending |
| Phase 6 | System integration testing | Pending |
| Phase 7 | Water trials and parameter identification | Pending |

---

## Document Control

| Version | Date | Author | Change |
|---|---|---|---|
| 1.0 | June 2026 | Systems Integration Team | Initial README — Phase 2 simulation environment |
