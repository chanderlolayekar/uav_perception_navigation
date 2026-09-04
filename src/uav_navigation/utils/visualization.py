import matplotlib.pyplot as plt 
import numpy as np 

class VisualizationModule: 
    def __init__(self): 
        self.fig, self.ax = plt.subplots(figsize=(8, 8)) 

    def plot_map(self, occupancy_grid): 
        self.ax.clear() 
        self.ax.imshow(occupancy_grid, cmap='gray', origin='lower') 
        self.ax.set_title('Occupancy / ESDF Map') 
        plt.pause(0.001) 

    def plot_trajectory(self, trajectory): 
        if trajectory.size == 0: 
            return 
        self.ax.plot(trajectory[:,0], trajectory[:,1], '-r', label='Planned Trajectory') 
        self.ax.legend() 
        plt.pause(0.001) 
