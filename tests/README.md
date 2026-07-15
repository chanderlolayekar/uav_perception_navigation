# Integration & Pipeline Testing

This directory contains integration test scripts designed to verify that the core modules of the UAV navigation and perception pipeline interface correctly.

These tests allow you to check the pipeline's logic locally on your computer without requiring a full ROS environment or physical hardware connection.

## Core Test Script

### `test_pipeline.py`
This script acts as a mock simulation runner. It performs the following integration checks:

1. **Perception Module Verification**: Initializes the `PerceptionQualityModule` and feeds it mock 3D LiDAR points and occupancy matrices to verify that occupancy confidence scores are computed without crashing.
2. **Optimizer Verification**: Instantiates a mock ESDF map (`DummyEsdfMap`) and passes a straight-line initial path into the `TrajectoryOptimizer` to verify that the SciPy L-BFGS-B optimization step converges and outputs a smoothed trajectory.
3. **Visualization Mocking**: Instantiates the `VisualizationModule` to ensure matplotlib handlers and plotting loops execute correctly.

## How to Run the Tests

To run the integration pipeline test, navigate to the root directory of the project (`uav_perception_navigation/`) and run the script as a Python module:

```bash
python -m tests.test_pipeline
