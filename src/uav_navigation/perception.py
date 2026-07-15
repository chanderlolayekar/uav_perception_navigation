
import numpy as np
import cv2 [cite: 274, 275]

class PerceptionQualityModule: [cite: 276]
    def __init__(self, occupancy_threshold=0.6, semantic_classes=None): [cite: 276]
        self.occupancy_threshold = occupancy_threshold [cite: 276]
        self.semantic_classes = semantic_classes if semantic_classes else ['background', 'obstacle', 'free'] [cite: 276]

    def compute_occupancy_confidence(self, lidar_pointcloud, occupancy_grid): [cite: 276]
        """
        Compute occupancy confidence from LIDAR data. 
        """
        grid_res = 0.1 # 10 cm resolution [cite: 278]
        confidence_values = [] [cite: 278]
        for point in lidar_pointcloud: [cite: 278]
            # Project LIDAR points to 2D occupancy map indices [cite: 278]
            x_idx = int(point[0] / grid_res)
            y_idx = int(point[1] / grid_res)
            if 0 <= x_idx < occupancy_grid.shape[0] and 0 <= y_idx < occupancy_grid.shape[1]: [cite: 279]
                confidence_values.append(occupancy_grid[x_idx, y_idx]) [cite: 279]
        if confidence_values: [cite: 279]
            confidence_score = np.mean(confidence_values) [cite: 279]
        else: [cite: 279]
            confidence_score = 0.0 [cite: 279, 280]
        return confidence_score [cite: 280]

    def compute_semantic_certainty(self, semantic_image): [cite: 280]
        """
        Compute semantic certainty from segmented camera image. [cite: 280]
        """
        total_pixels = semantic_image.size [cite: 281]
        non_background_pixels = np.sum(semantic_image != 0) [cite: 281]
        certainty_score = non_background_pixels / total_pixels [cite: 282]
        return certainty_score

    def compute_feature_richness(self, camera_image): [cite: 283]
        """
        Compute feature richness metric based on number of detected keypoints. [cite: 283]
        """
        gray_image = cv2.cvtColor(camera_image, cv2.COLOR_BGR2GRAY) [cite: 284]
        orb = cv2.ORB_create() [cite: 284]
        keypoints = orb.detect(gray_image, None) [cite: 284]
        max_keypoints = 1000 # arbitrary normalization factor [cite: 284]
        richness_score = min(len(keypoints) / max_keypoints, 1.0) [cite: 284]
        return richness_score [cite: 285]

    def fuse_metrics(self, occupancy_confidence, semantic_certainty, feature_richness, weights=[0.4, 0.3, 0.3]): [cite: 285]
        """
        Weighted fusion of perception metrics into a single score. [cite: 285, 286]
        """
        fused_score = (weights[0] * occupancy_confidence +
                       weights[1] * semantic_certainty +
                       weights[2] * feature_richness) [cite: 286]
        return fused_score [cite: 287]

if __name__ == "__main__": [cite: 287]
    pqm = PerceptionQualityModule() [cite: 287]
    lidar_points = np.random.uniform(0, 5, (1000, 3)) # 1000 points [cite: 287]
    occupancy_grid = np.random.uniform(0, 1, (100, 100)) # 10m x 10m map [cite: 287]
    semantic_seg_image = np.random.randint(0, 3, (480, 640)) [cite: 287]
    rgb_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8) [cite: 287, 288]

    occ_conf = pqm.compute_occupancy_confidence(lidar_points, occupancy_grid) [cite: 288]
    sem_cert = pqm.compute_semantic_certainty(semantic_seg_image) [cite: 288]
    feat_rich = pqm.compute_feature_richness(rgb_image) [cite: 288, 289]
    fused_quality = pqm.fuse_metrics(occ_conf, sem_cert, feat_rich) [cite: 289]

    print(f"Occupancy Confidence: {occ_conf:.3f}") [cite: 289]
    print(f"Semantic Certainty: {sem_cert:.3f}") [cite: 289]
    print(f"Feature Richness: {feat_rich:.3f}") [cite: 288, 289]
    print(f"Fused Perception Quality: {fused_quality:.3f}") [cite: 289]
