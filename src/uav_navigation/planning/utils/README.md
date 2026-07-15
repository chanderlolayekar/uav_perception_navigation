# Utility, Interface, & Visualization Modules

[cite_start]This directory manages communication with outer environments (e.g., ROS), parses input streams, logs telemetry, and displays results[cite: 3, 335, 345].

## Core Scripts

### 1. Simulation Interface (`simulation.py`)
[cite_start]Provides subscriber handlers mapping standard robotic sensor configurations into local structures[cite: 335]:
* [cite_start]**`/esdf_map`**: Parses real-time environmental occupancy and bounds mapping[cite: 336].
* **`/uav_pose`**: Captures spatial location metrics to monitor flight trajectories[cite: 336].
* **`/lidar_points`**: Captures distance sensor array returns[cite: 336].

### 2. Data Logger (`simulation.py`)
* Writes key telemetry elements (`timestamp`, `x`, `y`, `z`, `vx`, `vy`, `vz`) into a localized CSV file (`uav_log.csv`) at $10\text{ Hz}$[cite: 340, 347].
* [cite_start]Ideal for post-run performance validation and accuracy analysis[cite: 346].

### 3. Visualization Module (`visualization.py`)
* [cite_start]Renders real-time visual outputs using Matplotlib[cite: 345].
* Dynamically overlays the generated paths onto the generated occupancy maps to visually demonstrate the planning outputs[cite: 345].
