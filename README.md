# Autonomous Forest Navigation with Sensor Fusion, LiDAR Mapping, and A* Path Planning

An end-to-end robotics project for autonomous ground-robot navigation in a forest environment using multimodal sensor data from the FoMo dataset.

![Autonomous Forest Navigation](figures/green_astar_planned_path.png)

**EKF localization RMSE:** 0.749 m · **A* path:** 9.02 m · **Path efficiency:** 94.6%

The project combines:

- VectorNav IMU measurements
- Robot odometry
- Emlid GNSS quality information
- PPK reference localization
- 128-channel RoboSense LiDAR
- Extended Kalman Filter (EKF) sensor fusion
- Terrain-aware LiDAR segmentation
- Ray-traced occupancy mapping
- Robot-footprint obstacle inflation
- A* path planning

The complete pipeline is implemented as five reproducible Jupyter notebooks.

---

## Skills Demonstrated

**Robotics:** localization, sensor fusion, LiDAR perception, occupancy mapping, path planning  
**Algorithms:** Extended Kalman Filter (EKF), ray tracing, obstacle inflation, A* search, terrain segmentation  
**Python:** NumPy, Pandas, SciPy, Matplotlib, Boto3  
**Data Processing:** IMU, odometry, GNSS metadata, LiDAR point clouds

## Project Overview

Autonomous navigation in a forest is challenging because a robot must estimate its position while moving over uneven terrain, identify obstacles such as trees and vegetation, build a traversable map, and plan a safe path through partially observed space.

This project implements the following pipeline:

```text
IMU + Odometry + GNSS Quality
             │
             ▼
     Extended Kalman Filter
             │
             ▼
       Robot Localization
             │
             ▼
       RoboSense LiDAR
             │
             ▼
 Ground / Obstacle Segmentation
             │
             ▼
   Ray-Traced Occupancy Grid
             │
             ▼
   Robot Footprint Inflation
             │
             ▼
          A* Planner
             │
             ▼
 Collision-Aware Local Path
```

---

## Key Results

### Localization Performance

| Method | RMSE | Mean Error | Maximum Error |
|---|---:|---:|---:|
| Dead Reckoning | 301.471 m | 254.638 m | 574.731 m |
| GNSS Only | 1.095 m | 0.727 m | 18.905 m |
| **EKF Sensor Fusion** | **0.749 m** | **0.697 m** | **3.254 m** |

The EKF reduced localization RMSE by approximately:

- **99.75% compared with dead reckoning**
- **31.61% compared with the quality-aware GNSS stream**

![Localization comparison](figures/green_localization_comparison.png)

The fused localization estimate remains close to the PPK reference while dead reckoning accumulates substantial drift.

![EKF position error](figures/green_ekf_position_error.png)

---

## Sensor Fusion

The localization filter uses the state:

```text
[x, y, yaw]
```

Prediction inputs:

- Odometry-derived linear velocity
- VectorNav yaw-rate measurement `wz`

Correction input:

- Quality-aware GNSS position measurements

The nonlinear prediction model is:

```text
x(k+1)   = x(k) + v cos(yaw) Δt
y(k+1)   = y(k) + v sin(yaw) Δt
yaw(k+1) = yaw(k) + ω Δt
```

The EKF runs on the IMU timeline at approximately **200 Hz**.

GNSS corrections occur at approximately **10 Hz**.

### GNSS Quality Model

The dataset provides real Emlid RTK solution-quality metadata but does not provide a separate raw GNSS XYZ stream for this experiment.

Therefore, the GNSS position measurements used in this project are explicitly modeled as **PPK-derived simulated GNSS measurements**.

The position noise and availability are controlled using real Emlid receiver metadata:

| GNSS State | Position Noise Assumption |
|---|---:|
| RTK Fix | σ = 0.5 m |
| RTK Float | σ = 2.5 m |
| Single | σ = 7.0 m |

Additional uncertainty is applied during poor satellite availability, and measurements are dropped when satellite availability becomes extremely low.

The untouched PPK trajectory is used only for evaluation.

### GNSS Quality During the Forest Run

![GNSS quality](figures/green_emlid_gnss_quality.png)

![Satellite count](figures/green_emlid_satellite_count.png)

---

## Odometry Drift

Robot odometry alone accumulates substantial position drift over the forest trajectory.

![Odometry drift](figures/green_odometry_drift_trajectory.png)

The IMU and odometry angular motion remain strongly correlated, supporting the use of VectorNav yaw rate inside the EKF.

![IMU vs odometry yaw rate](figures/green_imu_vs_odometry_yaw_rate.png)

---

## LiDAR Processing

A representative scan from the RoboSense LiDAR stream was decoded and processed.

The scan contains:

```text
162,108 points
128 LiDAR rings
~0.10 s scan duration
```

The verified point layout is:

```text
x          float32
y          float32
z          float32
intensity  float32
ring_id    uint16
timestamp  uint64
```

### Raw Forest Point Cloud

![RoboSense bird's-eye view](figures/green_robosense_birdseye.png)

The scan contains uneven forest terrain, trees, vegetation, and vertical obstacles around the robot.

### Side View

![RoboSense side view](figures/green_robosense_sideview.png)

---

## Ground and Obstacle Segmentation

A local terrain model is built using a **0.50 m XY grid**.

Instead of using a single fixed Z threshold, the local terrain height is estimated using a robust lower percentile within each grid cell.

Points are classified by height above local terrain:

```text
0.00 – 0.45 m  → Ground / terrain
0.45 – 2.50 m  → Navigation obstacle
> 2.50 m       → High vegetation / canopy
```

For the representative scan:

```text
Total points            : 161,735
Likely ground           : 95,845
Navigation obstacles    : 64,210
High vegetation/canopy  : 1,680
```

### Navigation-Relevant Obstacles

![Navigation obstacles](figures/green_lidar_navigation_obstacles_bev.png)

### Ground / Obstacle Side View

![Ground obstacle side view](figures/green_lidar_navigation_obstacles_side.png)

---

## Occupancy Mapping

The segmented LiDAR points are converted into a 2D occupancy grid.

Map configuration:

```text
Map size          : 70 m × 70 m
Grid resolution   : 0.25 m/cell
Grid dimensions   : 280 × 280
```

Occupancy encoding:

```text
-1   Unknown
 0   Free
100  Occupied
```

### Ray Tracing

LiDAR rays are traced from the robot to observed ground and obstacle endpoints.

Cells crossed by valid LiDAR rays are marked as observed free space, while obstacle endpoints are marked occupied.

Unknown regions remain unknown instead of being incorrectly assumed free.

![Ray-traced occupancy map](figures/green_local_occupancy_grid_raytraced.png)

The robot-reachable region for the selected scan contains:

```text
Reachable free area : 38.31 m²
Maximum reach       : 10.14 m
```

---

## Robot Safety Inflation

For the prototype planner, the following configurable assumptions are used:

```text
Robot radius     : 0.50 m
Safety margin    : 0.25 m
Inflation radius : 0.75 m
```

Occupied cells are inflated before path planning so the robot's center does not pass too close to obstacles.

These dimensions are prototype planning assumptions rather than measured physical dimensions of the FoMo platform.

---

## A* Path Planning

A* is implemented from scratch using an **8-connected grid**.

Allowed motion:

```text
↑  ↓  ←  →
↖  ↗  ↙  ↘
```

Unknown and occupied cells are treated as non-traversable.

Diagonal corner cutting is also prohibited, preventing the planner from mathematically squeezing between touching obstacles.

### Automatic Goal Selection

A safe reachable goal is automatically selected from the robot-connected free-space region.

![A* start and goal](figures/green_astar_start_goal.png)

### Final Path Result

```text
Straight-line distance : 8.53 m
A* path length         : 9.02 m
Path efficiency        : 94.6%
Path cells             : 35
Expanded nodes         : 119
```

![A* planned path](figures/green_astar_planned_path.png)

The resulting path remains inside observed free space and avoids inflated obstacles.

---

## Repository Structure

```text
Autonomous-Forest-Navigation/
│
├── notebooks/
│   ├── 01_dataset_exploration.ipynb
│   ├── 02_sensor_fusion_ekf.ipynb
│   ├── 03_lidar_processing.ipynb
│   ├── 04_occupancy_mapping.ipynb
│   └── 05_path_planning.ipynb
│
├── figures/
│   └── generated project figures
│
├── outputs/
│   ├── green_localization_metrics.csv
│   └── path_planning/
│       ├── green_astar_metrics.csv
│       └── green_astar_waypoints.csv
│
├── requirements.txt
├── .gitignore
└── README.md
```

Large raw dataset files, the local Conda environment, and large regenerated intermediate outputs are intentionally excluded from Git.

---

## Notebook Pipeline

### 01 — Dataset Exploration

- Connect to the public FoMo dataset
- Inspect available deployments and sessions
- Download selected localization files
- Analyze PPK reference trajectory
- Analyze VectorNav IMU
- Analyze robot odometry
- Inspect real Emlid GNSS quality metadata
- Create reproducible quality-aware GNSS measurements

### 02 — Sensor Fusion EKF

- Synchronize IMU, odometry, and GNSS streams
- Estimate initial heading
- Implement nonlinear EKF prediction
- Perform GNSS corrections
- Evaluate against PPK ground truth
- Compare dead reckoning, GNSS-only, and EKF localization

### 03 — LiDAR Processing

- Inspect FoMo LiDAR streams
- Decode RoboSense binary point clouds
- Visualize forest geometry
- Estimate local terrain height
- Separate ground, obstacles, and high vegetation

### 04 — Occupancy Mapping

- Align LiDAR scan timestamp with EKF pose
- Construct local occupancy grid
- Ray-trace observed free space
- Preserve unknown areas
- Inflate obstacles for robot footprint
- Determine robot-reachable free space

### 05 — Path Planning

- Load the occupancy map
- Automatically choose a safe reachable goal
- Implement A* search
- Prevent diagonal corner cutting
- Generate metric waypoints and headings
- Save final path-planning results

---

## Installation

Python **3.11** is recommended.

Create a Conda environment:

```bash
conda create -n forest-nav python=3.11 -y
conda activate forest-nav
```

Install project dependencies:

```bash
pip install -r requirements.txt
```

Register the environment as a Jupyter kernel:

```bash
python -m ipykernel install --user --name forest-nav --display-name "Python (Forest Navigation)"
```

Start JupyterLab:

```bash
jupyter lab
```

Run the notebooks sequentially:

```text
01_dataset_exploration.ipynb
        ↓
02_sensor_fusion_ekf.ipynb
        ↓
03_lidar_processing.ipynb
        ↓
04_occupancy_mapping.ipynb
        ↓
05_path_planning.ipynb
```

---

## Dependencies

Core Python packages:

```text
NumPy
Pandas
Matplotlib
SciPy
Boto3
JupyterLab
IPython Kernel
```

See `requirements.txt` for version constraints.

---

## Dataset


This project uses the **FoMo: A Multi-Season Dataset for Robot Navigation in Forêt Montmorency** dataset developed by the Northern Robotics Laboratory (NorLab), Université Laval.

Official dataset website: https://fomo.norlab.ulaval.ca/


This project uses the following FoMo forest session:

```text
2025-06-26
green_2025-06-26-11-20
```

The project does **not** commit the FoMo dataset to this repository.

Selected data is downloaded locally during the workflow.

Only a representative RoboSense scan is downloaded for the LiDAR perception and local-planning demonstration rather than the complete multi-gigabyte LiDAR stream.

---

## Important Experimental Notes

### GNSS

The GNSS position stream used by the EKF is **not claimed to be raw GNSS XYZ directly supplied by the dataset**.

It is generated from the PPK reference trajectory using reproducible noise and dropout behavior controlled by real Emlid GNSS quality metadata.

The untouched PPK trajectory is used for evaluation.

### LiDAR Mapping

The current occupancy-map and A* demonstration use a **single representative RoboSense scan**.

Therefore, the path-planning result demonstrates local collision-aware navigation through currently observed forest space, not autonomous traversal of the complete recording.

### Robot Geometry

The robot radius and safety margin used for obstacle inflation are configurable prototype assumptions and are not claimed to be measured physical dimensions of the FoMo platform.

---

## Future Work

Possible extensions include:

- Multi-scan LiDAR accumulation using the EKF trajectory
- Global forest occupancy mapping
- LiDAR motion compensation / deskewing
- Plane-fitting or RANSAC-based ground segmentation
- Probabilistic log-odds occupancy mapping
- Dynamic replanning
- D* Lite or Hybrid A*
- Path smoothing and curvature constraints
- Robot motion-controller simulation
- ROS 2 integration
- Real raw GNSS integration when available
- Quantitative evaluation across additional FoMo sessions and seasons

---

## Project Takeaway

This project demonstrates a complete autonomous-navigation workflow using forest robotics data:

```text
Sensor synchronization
        ↓
Sensor fusion localization
        ↓
3D LiDAR perception
        ↓
Terrain / obstacle extraction
        ↓
Occupancy mapping
        ↓
Collision-aware path planning
```

The main focus is not only on individual algorithms, but on connecting localization, perception, mapping, and planning into one coherent robotics pipeline.