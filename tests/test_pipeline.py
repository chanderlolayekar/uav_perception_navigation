import numpy as np
from uav_navigation.perception import PerceptionQualityModule
from uav_navigation.planning.optimization import TrajectoryOptimizer, DummyEsdfMap, perception_quality
from uav_navigation.utils.visualization import VisualizationModule

def main():
    print("Testing local UAV Navigation Pipeline integration...")

    # 1. Test perception quality module
    pqm = PerceptionQualityModule()
    lidar_points = np.random.uniform(0, 5, (50, 3))
    occupancy_grid = np.random.uniform(0, 1, (100, 100))
    occ_conf = pqm.compute_occupancy_confidence(lidar_points, occupancy_grid)
    print(f"-> Local simulation: computed dummy occupancy confidence = {occ_conf:.3f}")

    # 2. Test trajectory optimization
    start = np.array([0, 0])
    goal = np.array([50, 50])
    initial_traj = np.linspace(start, goal, 10)

    esdf_map = DummyEsdfMap()
    optimizer = TrajectoryOptimizer(esdf_map, perception_quality)
    optimized_traj = optimizer.optimize(initial_traj)
    print("-> Trajectory optimizer ran successfully!")

if __name__ == "__main__":
    main()
