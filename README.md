<div align="center">

# Autonomous Drone

### Vision-Based Autonomous Exploration on a Lightweight Quadrotor

<p>
  <img src="https://img.shields.io/badge/ROS%202-Humble-22314E?style=for-the-badge&logo=ros" alt="ROS 2">
  <img src="https://img.shields.io/badge/Python-3.x-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python">
  <img src="https://img.shields.io/badge/C%2B%2B-17-00599C?style=for-the-badge&logo=cplusplus&logoColor=white" alt="C++">
  <img src="https://img.shields.io/badge/OpenCV-Computer%20Vision-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white" alt="OpenCV">
  <img src="https://img.shields.io/badge/ORB--SLAM3-Visual%20SLAM-2E8B57?style=for-the-badge" alt="ORB-SLAM3">
</p>

<p>
  <strong>
    A modular autonomous-drone stack combining monocular visual SLAM,
    learned depth, metric scale estimation, 3D occupancy mapping,
    exploration, and real-time flight control.
  </strong>
</p>

</div>

---

## Overview

**Autonomous Drone** is a robotics project built around a **Tello / Tello Talent** platform, with the goal of developing a vision-first autonomy system for aerial exploration.

The project combines a lightweight drone interface with a ROS 2-based perception and navigation stack. The system receives live visual data from the drone, estimates motion using **ORB-SLAM3**, recovers scene structure using **Depth Anything V2**, aligns the reconstruction to metric scale, builds a 3D occupancy representation, and uses that representation as the basis for exploration and mission-level decisions.

The architecture is designed so that the major parts of the system remain independent:

```text
                    Tello
                      │
                      ▼
              Drone Interface
                      │
                      ▼
                 Camera Feed
                      │
          ┌───────────┴───────────┐
          ▼                       ▼
      ORB-SLAM3            Depth Anything V2
          │                       │
          │ Pose                  │ Depth
          └───────────┬───────────┘
                      ▼
               Metric Alignment
                      │
                      ▼
              3D Occupancy Map
                      │
                      ▼
              Exploration / Search
                      │
                      ▼
                Mission Control
                      │
                      ▼
                   Tello
```

---

## Why This Project?

The project explores how far autonomous aerial navigation can be taken using a **small, lightweight drone and vision-based perception** rather than depending on a large sensor stack.

A monocular camera provides rich visual information at very low hardware cost, but it introduces difficult problems:

* Monocular scale ambiguity
* Visual drift
* Depth estimation
* Coordinate-frame consistency
* Sensor synchronization
* Real-time processing
* Reliable mapping from an airborne platform

This repository is built around solving those problems as a complete system rather than treating each one as an isolated experiment.

---

# System Architecture

The current implementation is organized around two closely connected areas:

```text
Autonomous_drone/
│
├── tello_autonomy/
│   ├── drone_interface/
│   ├── middleware/
│   ├── perception/
│   ├── goals/
│   ├── config/
│   ├── scripts/
│   └── main.py
│
└── ros2_test/
    └── src/
        ├── ros2_orb_slam3/
        ├── occupancy_map_cpp/
        ├── exploration_cpp/
        ├── search_cpp/
        ├── octomap_manager_shim/
        └── tello_autonomy_msgs/
```

The **`tello_autonomy` package is the main application layer**. Its `main.py` connects the drone interface, ROS 2 middleware, perception, scale estimation, ToF integration, diagnostics, and mission control into the running system.

The **`ros2_test/src` directory contains the active ROS 2 packages** used by the system.

---

# Core Pipeline

## 1. Drone Interface

The `tello_autonomy/drone_interface/` layer isolates the physical drone from the rest of the autonomy system.

The current implementation includes dedicated components for:

* Tello connection
* Video acquisition
* Telemetry
* Command handling
* Manual control
* External ToF communication

The main driver initializes the drone with configurable video resolution, frame rate, and bitrate, while telemetry and flight commands are handled independently.

```text
             Tello
               │
       ┌───────┼────────┐
       ▼       ▼        ▼
    Video   Telemetry  Commands
       │       │        │
       └───────┼────────┘
               ▼
       tello_autonomy
```

---

## 2. Low-Latency Video

The camera stream is the starting point for the perception pipeline.

Because autonomous flight depends on current observations, stale frames can be as problematic as inaccurate ones. The video pipeline therefore handles decoding and buffering with latency in mind.

```text
Tello H.264 Stream
        │
        ▼
      PyAV
        │
        ▼
Decoded Frame
        │
        ▼
Perception Pipeline
```

The driver configures the video pipeline with low-delay options and limits buffering to keep the working frame close to the current drone view.

---

# 3. Visual SLAM

Visual localization is provided by the **ORB-SLAM3** integration under:

```text
ros2_test/src/ros2_orb_slam3/
```

The SLAM component is kept as a separate ROS 2 process rather than being embedded directly inside the main Python application.

This separation gives the architecture a clean boundary:

```text
tello_autonomy
      │
      │ ROS 2
      ▼
┌─────────────────┐
│   ORB-SLAM3     │
│   C++ / ROS 2   │
└─────────────────┘
```

The main application explicitly connects to the independent SLAM process through ROS 2.

### What SLAM provides

* Camera pose
* Keyframe information
* Sparse visual geometry
* Motion estimation
* Spatial relationships over time

---

# 4. Monocular Depth

The perception pipeline also integrates **Depth Anything V2**.

Where visual SLAM provides a sparse representation and camera motion, the depth model provides dense scene-depth information.

```text
               RGB Frame
                   │
          ┌────────┴────────┐
          ▼                 ▼
      ORB-SLAM3       Depth Anything V2
          │                 │
          ▼                 ▼
        Pose              Depth
          │                 │
          └────────┬────────┘
                   ▼
              3D Geometry
```

This combination allows the system to move beyond sparse feature geometry toward a richer representation of the surrounding environment.

---

# 5. Metric Scale

A major limitation of monocular SLAM is that the recovered trajectory is not inherently metric.

The system therefore contains a dedicated scale-estimation pipeline:

```text
tello_autonomy/perception/
```

including components such as:

```text
scale_factor.py
scale_factor_manager.py
live_scaler.py
tof_scale_estimator.py
ext_tof_scale_estimator.py
```

The main application initializes these components directly.

The basic idea is:

```text
SLAM Coordinates
       │
       ▼
Scale Estimation
       │
       ▼
Metric Scale Factor
       │
       ▼
Scaled Pose / Points
```

This is essential because a physically meaningful map requires more than a visually consistent trajectory.

---

# 6. ToF-Assisted Scale

The project also includes a ToF-based scale estimation path.

A known physical distance can provide an external metric reference against the relative geometry produced by the monocular pipeline.

```text
      Visual Motion
           │
           ▼
    Relative Distance
           │
           ├──────────────┐
           │              │
           ▼              ▼
      SLAM Estimate     ToF
           │              │
           └──────┬───────┘
                  ▼
             Scale Factor
```

The repository contains dedicated ToF drivers, bridges, and scale-estimation components for this purpose.

A dedicated validation script is also included:

```text
tello_autonomy/validate_2m_factor.py
```

which is intended to evaluate scale against a known physical distance.

---

# 7. Live Scaling

Once a usable scale factor is available, the `LiveScaler` applies it to incoming pose and point data.

```text
ORB-SLAM3
    │
    ▼
Relative Pose
    │
    ▼
Current Scale
    │
    ▼
Scaled Pose
    │
    ▼
Metric Coordinate System
```

The main application creates the scale manager and live scaler together as part of the perception pipeline.

---

# 8. 3D Occupancy Mapping

The active ROS 2 mapping implementation is located in:

```text
ros2_test/src/occupancy_map_cpp/
```

This package provides the mapping layer used by the autonomy system.

The purpose is to transform geometric information into a representation of:

```text
Free Space
Occupied Space
Unknown Space
```

which can then be consumed by higher-level exploration logic.

```text
Depth + Pose
     │
     ▼
3D Points
     │
     ▼
Occupancy Representation
     │
     ├── Free
     ├── Occupied
     └── Unknown
```

For an aerial robot, maintaining a 3D representation is particularly important because the vehicle is not constrained to a planar ground path.

---

# 9. OctoMap Integration

The ROS 2 workspace also contains:

```text
ros2_test/src/octomap_manager_shim/
```

which provides the OctoMap-side integration used by the mapping and exploration stack.

This provides a clean interface between the occupancy representation and components that need to consume the map.

---

# 10. Exploration

The active exploration implementation is:

```text
ros2_test/src/exploration_cpp/
```

This is the current exploration package in the repository.

Its role is to operate on the current environment representation and determine useful exploration targets.

Conceptually:

```text
Current Map
    │
    ▼
Unknown / Unexplored Space
    │
    ▼
Candidate Targets
    │
    ▼
Exploration Decision
    │
    ▼
Mission Controller
```

The goal is to move away from predetermined motion and toward **map-driven exploration**.

---

# 11. Search

The search component is maintained separately under:

```text
ros2_test/src/search_cpp/
```

This keeps search and candidate-selection logic separate from the occupancy-map implementation.

That separation makes it possible to change exploration strategy without rewriting the mapping pipeline.

---

# 12. Mission Control

The main `tello_autonomy` application includes:

```text
goals/
├── mission_controller.py
└── trajectory_tracker.py
```

The `MissionControllerNode` is instantiated directly by the main application, linking high-level goals with the running autonomy system.

This creates a clean distinction between:

```text
Perception
    ↓
Environment Understanding
    ↓
Mission / Goal Decision
    ↓
Trajectory
    ↓
Flight Control
```

---

# ROS 2 Communication

The middleware layer lives in:

```text
tello_autonomy/middleware/
```

and currently includes:

```text
ros_bridge.py
message_types.py
topic_manager.py
service_manager.py
telemetry_bridge.py
ext_tof_bridge.py
```

These components provide the communication boundary between the main application and the ROS 2 computation graph.

The architecture therefore separates:

```text
Application Logic
        │
        │ ROS 2 Topics / Services
        ▼
ROS 2 Perception + Mapping + Exploration
```

---

# Concurrency Model

The main application deliberately gives each ROS 2 node its own `SingleThreadedExecutor` and spin thread.

This allows components such as:

* Scale estimation
* Live scaling
* ToF processing
* Global-map diagnostics
* Mission control

to operate independently without relying on one shared executor.

At the same time, manual flight control remains independent of the ROS 2 executor, so the drone interface does not have to wait for the SLAM or perception side of the system to become ready.

---

# Architecture at a Glance

```text
                           ┌───────────────┐
                           │     Tello     │
                           └───────┬───────┘
                                   │
                    ┌──────────────┼──────────────┐
                    │              │              │
                  Video        Telemetry       Control
                    │              │              │
                    └──────────────┼──────────────┘
                                   ▼
                         ┌──────────────────┐
                         │ tello_autonomy   │
                         │   Main Package   │
                         └─────────┬────────┘
                                   │
                ┌──────────────────┼──────────────────┐
                │                  │                  │
                ▼                  ▼                  ▼
          Drone Interface      Middleware       Perception
                                   │                  │
                                   ▼          ┌───────┴───────┐
                               ROS 2 Graph    │               │
                                              ▼               ▼
                                         ORB-SLAM3        Depth V2
                                              │               │
                                              └───────┬───────┘
                                                      ▼
                                               Scale / Fusion
                                                      │
                                                      ▼
                                              Occupancy Mapping
                                                      │
                                                      ▼
                                           Exploration / Search
                                                      │
                                                      ▼
                                              Mission Control
                                                      │
                                                      ▼
                                                   Tello
```

---

# Repository Structure

```text
Autonomous_drone/
│
├── tello_autonomy/
│   │
│   ├── config/
│   │
│   ├── drone_interface/
│   │   ├── command_handler.py
│   │   ├── ext_tof_driver.py
│   │   ├── frame_receiver.py
│   │   ├── manual_control.py
│   │   ├── telemetry.py
│   │   └── tello_driver.py
│   │
│   ├── goals/
│   │   ├── mission_controller.py
│   │   └── trajectory_tracker.py
│   │
│   ├── middleware/
│   │   ├── ext_tof_bridge.py
│   │   ├── message_types.py
│   │   ├── ros_bridge.py
│   │   ├── service_manager.py
│   │   ├── telemetry_bridge.py
│   │   └── topic_manager.py
│   │
│   ├── perception/
│   │   ├── depth/
│   │   ├── camera_intrinsics.py
│   │   ├── depth_backprojection.py
│   │   ├── depth_inference_worker.py
│   │   ├── ext_tof_scale_estimator.py
│   │   ├── global_map_diagnostic_exporter.py
│   │   ├── live_scaler.py
│   │   ├── pose_transform.py
│   │   ├── scale_factor.py
│   │   ├── scale_factor_manager.py
│   │   └── tof_scale_estimator.py
│   │
│   ├── scripts/
│   │
│   ├── main.py
│   └── validate_2m_factor.py
│
├── ros2_test/
│   │
│   └── src/
│       ├── exploration_cpp/
│       ├── occupancy_map_cpp/
│       ├── octomap_manager_shim/
│       ├── ros2_orb_slam3/
│       ├── search_cpp/
│       └── tello_autonomy_msgs/
│
├── scripts/
│
├── data.py
├── global_keyframe_pixels.csv
├── global_map_diagnostics.json
├── test_cv2_stream.py
└── .gitignore
```

The key distinction is:

> **`tello_autonomy/` is the main application package; `ros2_test/src/` contains the active ROS 2 implementation that supports it.**

---

# Technology Stack

| Layer            | Technology                  |
| ---------------- | --------------------------- |
| Drone            | Tello / Tello Talent        |
| Main Application | Python                      |
| ROS 2 Packages   | C++ / Python                |
| Middleware       | ROS 2                       |
| Visual SLAM      | ORB-SLAM3                   |
| Depth Estimation | Depth Anything V2           |
| Computer Vision  | OpenCV                      |
| 3D Mapping       | OctoMap / Occupancy Mapping |
| Drone Interface  | `djitellopy`                |
| Video            | H.264 / PyAV / FFmpeg       |
| Visualization    | RViz                        |
| Build System     | CMake / `colcon`            |

---

# Getting Started

## Requirements

The project is intended for a Linux-based ROS 2 development environment.

You will need:

* Ubuntu
* ROS 2
* Python 3
* C++ compiler
* CMake
* `colcon`
* OpenCV
* Tello communication dependencies
* ORB-SLAM3 dependencies
* Depth Anything V2 environment

---

## Clone

```bash
git clone https://github.com/HamdanTariq26/Autonomous_drone.git
cd Autonomous_drone
```

---

## ROS 2 Workspace

The active ROS 2 packages are located under:

```text
ros2_test/src/
```

Create or use a ROS 2 workspace and place the packages inside the workspace `src` directory.

Then install dependencies with:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build using:

```bash
colcon build --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

---

# Running the System

The main application entry point is:

```text
tello_autonomy/main.py
```

The SLAM node is a separate ROS 2 process.

Start the SLAM node:

```bash
ros2 run ros2_orb_slam3 mono_node_cpp \
    --ros-args \
    -p node_name_arg:=mono_slam_cpp
```

Then launch the main autonomy application from the project environment.

The current `main.py` initializes:

```text
TelloDriver
FrameReceiver
TelemetryMonitor
CommandHandler
ManualControl
ExtTofDriver
RosBridge
TelemetryBridge
ExtTofBridge
ScaleFactorManager
LiveScaler
ToFScaleEstimator
ExtTofScaleEstimatorNode
GlobalMapDiagnosticExporter
MissionControllerNode
```

before starting the corresponding execution threads.

---

# Development Workflow

The project is developed incrementally, with individual subsystems validated before being relied upon by higher-level autonomy.

A typical progression is:

```text
Camera
  ↓
Drone Interface
  ↓
SLAM
  ↓
Depth
  ↓
Scale
  ↓
3D Mapping
  ↓
Exploration
  ↓
Mission Control
  ↓
Flight
```

This makes it possible to isolate problems instead of debugging the complete system as one monolithic program.

---

# Validation & Diagnostics

The repository contains dedicated utilities for validating individual parts of the pipeline.

Examples include:

```text
validate_2m_factor.py
```

along with perception and mapping diagnostics such as:

```text
global_map_diagnostic_exporter.py
```

and the repository's diagnostic data files.

These tools are used to investigate issues such as:

* Scale drift
* Incorrect transforms
* Map inconsistencies
* Keyframe geometry
* ToF measurements
* Occupancy-map behaviour
* Pipeline synchronization

This validation layer is important because a visually plausible result is not necessarily a metrically correct one.

---

# Current Development Focus

The project is still under active development.

The main engineering effort is now centered on bringing the perception, mapping, exploration, and mission-control layers together into a reliable closed-loop system.

Current areas include:

* Improving monocular scale stability
* Refining depth-to-3D projection
* Maintaining consistent coordinate frames
* Reducing perception latency
* Improving occupancy-map consistency
* Connecting exploration with trajectory generation
* Increasing robustness during real flight
* Handling tracking and perception failures

---

# Roadmap

### Perception

* [x] Tello video pipeline
* [x] Monocular SLAM integration
* [x] Depth estimation
* [x] Scale estimation
* [x] Live pose scaling
* [x] ToF integration
* [ ] Further scale stabilization
* [ ] More robust depth fusion

### Mapping

* [x] 3D occupancy representation
* [x] OctoMap integration
* [x] Map diagnostics
* [ ] Improved map consistency
* [ ] Better outlier handling
* [ ] Dynamic obstacle support

### Exploration

* [x] Exploration package
* [x] Search package
* [x] Mission-controller layer
* [ ] More robust target selection
* [ ] Improved replanning
* [ ] Better collision-aware exploration

### Autonomy

* [x] Drone communication
* [x] ROS 2 integration
* [x] Perception pipeline
* [x] Mapping infrastructure
* [ ] Closed-loop exploration
* [ ] Robust autonomous navigation
* [ ] End-to-end autonomous missions

---

# Limitations

The current system is experimental and several challenges remain inherent to vision-based aerial navigation.

### Monocular Scale

Scale estimation is an additional problem because the primary visual localization system is monocular.

### Computational Cost

SLAM, learned depth, mapping, and planning compete for compute resources.

### Visual Conditions

Tracking quality can degrade under difficult lighting, motion blur, low-texture scenes, or rapidly changing viewpoints.

### Real-World Flight

Physical flight introduces disturbances and communication delays that are difficult to reproduce perfectly in software.

---

# Safety

This software controls a physical aircraft and should be tested conservatively.

Recommended practice:

```text
Simulation / Offline Testing
            ↓
Bench Testing
            ↓
Manual Flight
            ↓
Perception-Only Testing
            ↓
Controlled Autonomous Motion
            ↓
Expanded Environment Testing
```

Maintain a reliable manual-control path throughout physical testing.

---

# Project Status

| Component                   | Status                |
| --------------------------- | --------------------- |
| Tello Interface             | 🟢 Active             |
| Video Pipeline              | 🟢 Active             |
| ORB-SLAM3                   | 🟢 Integrated         |
| Depth Anything V2           | 🟢 Integrated         |
| Metric Scaling              | 🟡 Refinement         |
| ToF Integration             | 🟢 Active             |
| Occupancy Mapping           | 🟢 Active             |
| Exploration                 | 🟡 Active Development |
| Mission Control             | 🟡 Active Development |
| Full Autonomous Exploration | 🟠 In Progress        |

---

# Why This Project Matters

The interesting part of this project is not any one individual component.

It is the attempt to make several very different systems cooperate on a small aerial platform:

```text
                    Vision
                       │
                 ┌─────┴─────┐
                 │           │
                SLAM       Depth
                 │           │
                 └─────┬─────┘
                       │
                     Scale
                       │
                       ▼
                    Mapping
                       │
                       ▼
                  Exploration
                       │
                       ▼
                     Motion
                       │
                       ▼
                     Drone
```

A successful autonomous system requires all of these layers to agree about **time, scale, coordinate frames, geometry, and the physical state of the drone**.

That systems-level integration is the central challenge of this repository.

---

# Author

**Hamdan Tariq**

BSc Artificial Intelligence

Focused on:

`Autonomous Systems` · `Robotics` · `Computer Vision` · `SLAM` · `Aerial Navigation`

---

# License

See the repository's `LICENSE` file for licensing information.

Third-party software and research components remain subject to their respective licenses.

---

<div align="center">

### From pixels to perception.

### From perception to maps.

### From maps to decisions.

### From decisions to flight.

**Autonomous Drone — an ongoing vision-first robotics project.**

</div>
