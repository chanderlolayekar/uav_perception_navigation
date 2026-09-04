
import numpy as np
import cv2 

class PerceptionQualityModule: 
    def __init__(self, occupancy_threshold=0.6, semantic_classes=None): 
        self.occupancy_threshold = occupancy_threshold 
        self.semantic_classes = semantic_classes if semantic_classes else ['background', 'obstacle', 'free']

    def compute_occupancy_confidence(self, lidar_pointcloud, occupancy_grid): 
        """
        Compute occupancy confidence from LIDAR data. 
        """
        grid_res = 0.1 # 10 cm resolution 
        confidence_values = [] 
        for point in lidar_pointcloud: 
            # Project LIDAR points to 2D occupancy map indices 
            x_idx = int(point[0] / grid_res)
            y_idx = int(point[1] / grid_res)
            if 0 <= x_idx < occupancy_grid.shape[0] and 0 <= y_idx < occupancy_grid.shape[1]: 
                confidence_values.append(occupancy_grid[x_idx, y_idx]) 
        if confidence_values: 
            confidence_score = np.mean(confidence_values)
        else: 
            confidence_score = 0.0 
        return confidence_score 

    def compute_semantic_certainty(self, semantic_image): 
        """
        Compute semantic certainty from segmented camera image. 
        """
        total_pixels = semantic_image.size 
        non_background_pixels = np.sum(semantic_image != 0) 
        certainty_score = non_background_pixels / total_pixels 
        return certainty_score

    def compute_feature_richness(self, camera_image): 
        """
        Compute feature richness metric based on number of detected keypoints. 
        """
        gray_image = cv2.cvtColor(camera_image, cv2.COLOR_BGR2GRAY) 
        orb = cv2.ORB_create() 
        keypoints = orb.detect(gray_image, None) 
        max_keypoints = 1000 # arbitrary normalization factor 
        richness_score = min(len(keypoints) / max_keypoints, 1.0) [
        return richness_score 

    def fuse_metrics(self, occupancy_confidence, semantic_certainty, feature_richness, weights=[0.4, 0.3, 0.3]): 
        """
        Weighted fusion of perception metrics into a single score. 
        """
        fused_score = (weights[0] * occupancy_confidence +
                       weights[1] * semantic_certainty +
                       weights[2] * feature_richness) 
        return fused_score 

if __name__ == "__main__": 
    pqm = PerceptionQualityModule() 
    lidar_points = np.random.uniform(0, 5, (1000, 3)) # 1000 points 
    occupancy_grid = np.random.uniform(0, 1, (100, 100)) # 10m x 10m map 
    semantic_seg_image = np.random.randint(0, 3, (480, 640)) 
    rgb_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8) 

    occ_conf = pqm.compute_occupancy_confidence(lidar_points, occupancy_grid) 
    sem_cert = pqm.compute_semantic_certainty(semantic_seg_image) 
    feat_rich = pqm.compute_feature_richness(rgb_image) 
    fused_quality = pqm.fuse_metrics(occ_conf, sem_cert, feat_rich) 

    print(f"Occupancy Confidence: {occ_conf:.3f}") 
    print(f"Semantic Certainty: {sem_cert:.3f}") 
    print(f"Feature Richness: {feat_rich:.3f}")
    print(f"Fused Perception Quality: {fused_quality:.3f}")
