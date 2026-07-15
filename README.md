# UAV Perception-Aware Navigation & Trajectory Optimization

This repository implements an integrated perception-aware navigation pipeline for autonomous Unmanned Aerial Vehicles (UAVs). It combines real-time incremental mapping, dynamic path planning, multi-sensor perception evaluation, and trajectory refinement.

## Project Architecture

```text
uav_perception_navigation/
├── README.md                          # Main documentation
├── .gitignore                         # Build and cache ignore rules
├── requirements.txt                   # Dependency list
├── src/
│   └── uav_navigation/
│       ├── __init__.py
│       ├── mapping.py                 # ESDF Map Generation
│       ├── perception.py              # Camera & LIDAR Perception Metrics
│       ├── planning/
│       │   ├── README.md              # Planner module documentation
│       │   ├── __init__.py
│       │   ├── kinodynamic.py         # Standard OMPL RRT*
│       │   ├── perception_aware.py    # Perception-weighted RRT*
│       │   └── optimization.py        # Gradient-based trajectory refiner
│       └── utils/
│           ├── README.md              # Simulation & Logging documentation
│           ├── __init__.py
│           ├── simulation.py          # ROS Interface & Pose Data Logger
│           └── visualization.py       # Live Matplotlib plotter
└── tests/
    └── test_pipeline.py               # Integration validation script
