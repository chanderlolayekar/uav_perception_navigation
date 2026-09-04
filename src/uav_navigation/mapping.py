import numpy as np
from fiesta import EsdfServer, EsdfIntegrator # Hypothetical python bindings for FIESTA 
import sensor_msgs.point_cloud2 as pc2 # For ROS point cloud processing 
import rospy
from sensor_msgs.msg import PointCloud2 

class EsdfMappingNode: 
    def __init__(self): 
        # Initialize ROS node
        rospy.init_node('esdf_mapping_node') 
        # Initialize ESDF Server with parameters
        self.esdf_server = EsdfServer(resolution=0.1, map_size=50) # 10cm voxel, 50m cubical 
        # Point cloud subscriber from simulated LIDAR
        self.pointcloud_sub = rospy.Subscriber('/lidar_points', PointCloud2, self.pointcloud_callback) 
        # Store latest point cloud for processing in callback
        self.latest_points = None 

    def pointcloud_callback(self, msg): 
        # Convert ROS PointCloud2 message to numpy array of points
        points = [] 
        for p in pc2.read_points(msg, skip_nans=True):
            points.append([p[0], p[1], p[2]]) 
        self.latest_points = np.array(points) 

    def update_esdf_map(self): 
        if self.latest_points is None: 
            return 
        # Integrate the latest point cloud into the ESDF map
        self.esdf_server.integrate_pointcloud(self.latest_points) 
        # Update ESDF distance field incrementally
        self.esdf_server.update_esdf() 

    def get_esdf_distance(self, position): 
        # Query distance to nearest obstacle at given position
        return self.esdf_server.get_distance_at(position)

    def spin(self): 
        rate = rospy.Rate(10) # 10 Hz update rate 
        while not rospy.is_shutdown(): 
            self.update_esdf_map() 
            rate.sleep() 

if __name__ == "__main__":
    esdf_node = EsdfMappingNode() 
    print("Starting ESDF mapping node...") 
    esdf_node.spin() 
