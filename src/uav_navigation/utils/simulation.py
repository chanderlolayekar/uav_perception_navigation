import rospy 
from nav_msgs.msg import OccupancyGrid 
from geometry_msgs.msg import PoseStamped 
from sensor_msgs.msg import PointCloud2 
import csv 
import numpy as np 

class SimulationInterface: 
    def __init__(self): 
        rospy.init_node('uav_interface', anonymous=True) 
        self.map_data = None 
        self.pose = None 
        self.pointcloud = None 
        rospy.Subscriber('/esdf_map', OccupancyGrid, self.map_callback) 
        rospy.Subscriber('/uav_pose', PoseStamped, self.pose_callback) 
        rospy.Subscriber('/lidar_points', PointCloud2, self.pc_callback) 

    def map_callback(self, msg): 
        self.map_data = msg 

    def pose_callback(self, msg): 
        self.pose = msg 

    def pc_callback(self, msg): 
        self.pointcloud = msg 

class DataLogger: 
    def __init__(self, filename='uav_log.csv'): 
        self.filename = filename 
        with open(self.filename, 'w', newline='') as csvfile: 
            writer = csv.writer(csvfile) 
            writer.writerow(['timestamp', 'x', 'y', 'z', 'vx', 'vy', 'vz']) 

    def log_pose(self, pose_stamped): 
        pos = pose_stamped.pose.position 
        vx, vy, vz = 0.0, 0.0, 0.0 
        with open(self.filename, 'a', newline='') as csvfile: 
            writer = csv.writer(csvfile) 
            writer.writerow([pose_stamped.header.stamp.to_sec(), pos.x, pos.y, pos.z, vx, vy, vz]) 
