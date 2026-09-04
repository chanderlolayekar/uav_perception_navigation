import numpy as np
from scipy.optimize import minimize 

class TrajectoryOptimizer:
    def __init__(self, esdf_map, perception_quality_func):
        self.esdf_map = esdf_map
        self.perception_quality_func = perception_quality_func 

    def objective(self, trajectory_flat): 
        traj = trajectory_flat.reshape(-1, 2) 
        collision_cost = 0.0 
        smoothness_cost = 0.0 
        perception_cost = 0.0 

        for i, pt in enumerate(traj):
            dist = self.esdf_map.distance(pt) 
            if dist < 0.5: 
                collision_cost += (0.5 - dist)**2 
            p_quality = self.perception_quality_func(pt) 
            perception_cost += (1.0 - p_quality)**2 

            if 1 <= i < len(traj) - 1: 
                prev_pt = traj[i - 1] 
                next_pt = traj[i + 1] 
                second_derivative = next_pt - 2 * pt + prev_pt 
                smoothness_cost += np.sum(second_derivative**2) 

        w_col = 10.0 
        w_smooth = 1.0 
        w_perc = 5.0 
        total_cost = w_col * collision_cost + w_smooth * smoothness_cost + w_perc * perception_cost 
        return total_cost 

    def optimize(self, initial_trajectory): 
        x0 = initial_trajectory.flatten() 
        result = minimize(self.objective, x0, method='L-BFGS-B', options={'maxiter': 200}) 
        optimized_traj = result.x.reshape(-1, 2) 
        return optimized_traj 

class DummyEsdfMap: 
    def distance(self, point): 
        center = np.array([25, 25]) 
        return np.linalg.norm(point - center) - 5.0 

    def gradient(self, point): 
        center = np.array([25, 25]) 
        diff = point - center 
        norm = np.linalg.norm(diff) 
        if norm == 0: 
            return np.array([0.0, 0.0]) 
        return diff / norm 

def perception_quality(point): 
    center = np.array([25, 25]) 
    dist_sq = np.sum((point - center)**2) 
    return np.exp(-dist_sq / 100) 

if __name__ == "__main__": 
    N = 20 
    start = np.array([0, 0]) 
    goal = np.array([50, 50]) 
    initial_traj = np.linspace(start, goal, N) 
    esdf_map = DummyEsdfMap()
    optimizer = TrajectoryOptimizer(esdf_map, perception_quality)
    optimized_traj = optimizer.optimize(initial_traj) 
    print("Initial trajectory:\n", initial_traj) 
    print("Optimized trajectory:\n", optimized_traj) 
