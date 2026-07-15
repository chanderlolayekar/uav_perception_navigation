import numpy as np
from scipy.optimize import minimize [cite: 323]

class TrajectoryOptimizer:
    def __init__(self, esdf_map, perception_quality_func):
        self.esdf_map = esdf_map
        self.perception_quality_func = perception_quality_func [cite: 316]

    def objective(self, trajectory_flat): [cite: 316]
        traj = trajectory_flat.reshape(-1, 2) [cite: 316, 317]
        collision_cost = 0.0 [cite: 317]
        smoothness_cost = 0.0 [cite: 317]
        perception_cost = 0.0 [cite: 317]

        for i, pt in enumerate(traj):
            dist = self.esdf_map.distance(pt) [cite: 317]
            if dist < 0.5: [cite: 318]
                collision_cost += (0.5 - dist)**2 [cite: 318]
            p_quality = self.perception_quality_func(pt) [cite: 318]
            perception_cost += (1.0 - p_quality)**2 [cite: 318, 319]

            if 1 <= i < len(traj) - 1: [cite: 320]
                prev_pt = traj[i - 1] [cite: 320]
                next_pt = traj[i + 1] [cite: 320]
                second_derivative = next_pt - 2 * pt + prev_pt [cite: 320, 321]
                smoothness_cost += np.sum(second_derivative**2) [cite: 321]

        w_col = 10.0 [cite: 321]
        w_smooth = 1.0 [cite: 321]
        w_perc = 5.0 [cite: 321]
        total_cost = w_col * collision_cost + w_smooth * smoothness_cost + w_perc * perception_cost [cite: 321, 322]
        return total_cost [cite: 322]

    def optimize(self, initial_trajectory): [cite: 322]
        x0 = initial_trajectory.flatten() [cite: 323]
        result = minimize(self.objective, x0, method='L-BFGS-B', options={'maxiter': 200}) [cite: 323]
        optimized_traj = result.x.reshape(-1, 2) [cite: 324]
        return optimized_traj [cite: 324]

class DummyEsdfMap: [cite: 324]
    def distance(self, point): [cite: 324]
        center = np.array([25, 25]) [cite: 324]
        return np.linalg.norm(point - center) - 5.0 [cite: 324]

    def gradient(self, point): [cite: 324]
        center = np.array([25, 25]) [cite: 325]
        diff = point - center [cite: 325]
        norm = np.linalg.norm(diff) [cite: 325]
        if norm == 0: [cite: 325]
            return np.array([0.0, 0.0]) [cite: 325]
        return diff / norm [cite: 325]

def perception_quality(point): [cite: 325]
    center = np.array([25, 25]) [cite: 325]
    dist_sq = np.sum((point - center)**2) [cite: 325, 326]
    return np.exp(-dist_sq / 100) [cite: 326]

if __name__ == "__main__": [cite: 326]
    N = 20 [cite: 327]
    start = np.array([0, 0]) [cite: 327]
    goal = np.array([50, 50]) [cite: 327]
    initial_traj = np.linspace(start, goal, N) [cite: 327]
    esdf_map = DummyEsdfMap() [cite: 327]
    optimizer = TrajectoryOptimizer(esdf_map, perception_quality) [cite: 327]
    optimized_traj = optimizer.optimize(initial_traj) [cite: 327]
    print("Initial trajectory:\n", initial_traj) [cite: 327]
    print("Optimized trajectory:\n", optimized_traj) [cite: 327]
