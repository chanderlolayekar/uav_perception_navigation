import matplotlib.pyplot as plt [cite: 335]
import numpy as np [cite: 335]

class VisualizationModule: [cite: 337]
    def __init__(self): [cite: 337]
        self.fig, self.ax = plt.subplots(figsize=(8, 8)) [cite: 337]

    def plot_map(self, occupancy_grid): [cite: 337]
        self.ax.clear() [cite: 337]
        self.ax.imshow(occupancy_grid, cmap='gray', origin='lower') [cite: 338]
        self.ax.set_title('Occupancy / ESDF Map') [cite: 338]
        plt.pause(0.001) [cite: 338]

    def plot_trajectory(self, trajectory): [cite: 338]
        if trajectory.size == 0: [cite: 338]
            return [cite: 338]
        self.ax.plot(trajectory[:,0], trajectory[:,1], '-r', label='Planned Trajectory') [cite: 338]
        self.ax.legend() [cite: 338]
        plt.pause(0.001) [cite: 339]
