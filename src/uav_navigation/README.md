# Core UAV Navigation & Perception Modules

This directory contains the primary modules responsible for environmental mapping and real-time perception quality assessment.

## Module Breakdown

### 1. Incremental ESDF Mapping (`mapping.py`)
* **Class**: `EsdfMappingNode`
* **Purpose**: Build a local, real-time 3D Euclidean Signed Distance Field (ESDF) map using incoming point clouds. This is crucial for fast collision checking and gradient-based trajectory optimization.
* **How it works**:
  1. Subscribes to the simulated LiDAR topic `/lidar_points` to receive `sensor_msgs/PointCloud2` data.
  2. Converts the point cloud data into a structured NumPy array.
  3. Uses a fast incremental ESDF server (like FIESTA) to update obstacle distances on the fly.
  4. Provides an interface (`get_esdf_distance(position)`) to query the exact distance to the nearest obstacle from any coordinate.

---

### 2. Perception Quality Module (`perception.py`)
* **Class**: `PerceptionQualityModule`
* **Purpose**: Evaluates how well the UAV can "see" and localize itself in its current environment. It generates a safety and localization confidence score by fusing three metrics:
  
#### Core Metrics Evaluated:
1. **Occupancy Confidence**: Projects 3D LiDAR point clouds onto a 2D occupancy grid to determine how reliably the space is mapped.
2. **Semantic Certainty**: Analyzes segmented camera images to calculate the ratio of recognized environment features (like obstacles vs. free space) to unknown areas.
3. **Feature Richness**: Runs OpenCV ORB (Oriented FAST and Rotated BRIEF) keypoint detection on camera images. More visual features (high texture, distinct corners) mean better visual odometry and localization accuracy.

#### Weighted Fusion:
The module combines these three values into a single score between `0.0` (blind/unstable) and `1.0` (excellent visibility and localization tracking):
$$\text{Fused Score} = (w_1 \times \text{Occupancy}) + (w_2 \times \text{Semantic}) + (w_3 \times \text{Features})$$
