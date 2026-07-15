# Utility, Interface, & Visualization Modules

[cite_start]This directory manages communication with outer environments (e.g., ROS), parses input streams, logs telemetry, and displays results.

## Core Scripts

### 1. Simulation Interface (`simulation.py`)
[cite_start]Provides subscriber handlers mapping standard robotic sensor configurations into local structures
* [cite_start]**`/esdf_map`**: Parses real-time environmental occupancy and bounds mapping.
* **`/uav_pose`**: Captures spatial location metrics to monitor flight trajectories.
* **`/lidar_points`**: Captures distance sensor array returns.

### 2. Data Logger (`simulation.py`)
* Writes key telemetry elements (`timestamp`, `x`, `y`, `z`, `vx`, `vy`, `vz`) into a localized CSV file (`uav_log.csv`) at $10\text{ Hz}$.
* [cite_start]Ideal for post-run performance validation and accuracy analysis.

### 3. Visualization Module (`visualization.py`)
* [cite_start]Renders real-time visual outputs using Matplotlib.
* Dynamically overlays the generated paths onto the generated occupancy maps to visually demonstrate the planning outputs.
