import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
import time
import open3d as o3d
import pandas as pd


class LidarFilter:
    def __init__(self):
        self.cluster_model = DBSCAN(eps=25, min_samples=15)
        self.filter_tests = [ self.height_test, self.length_test, self.width_test, self.number_of_pts_test]
        self.points = None
        self.ground_points = None
        self.clusters_list = None
        self.height_ground = None
        self.non_ground = None
        self.centers = None
        self.ground_model = RANSACRegressor(max_trials=500, residual_threshold=5)
        self.component_time_list = []
        
    @staticmethod
    def model_valid(model, X, y):
        # Angle from vertical
        coeffs = model.coef_
        norm = np.append(coeffs, [-1])
        angle_from_vertical = np.arccos(np.abs(norm[2]) / np.linalg.norm(norm))
        max_angle = np.pi / 4  # 45 degrees

        if angle_from_vertical > max_angle:
            return False

        # Inlier ratio check (only if inlier_mask_ exists)
        if hasattr(model, 'inlier_mask_'):
            inlier_ratio = np.sum(model.inlier_mask_) / len(y)
            print(f"Inlier ratio: {inlier_ratio}")
            if inlier_ratio < 0.5:
                return False

        return True


    def length_test(self, points, min_val=10, max_val=80):
        x_coords = points[:, 0]
        if min_val < np.abs(np.max(x_coords) - np.min(x_coords)) < max_val:
            return True
        return False

    def width_test(self, points, min_val=5, max_val=80):
        y_coords = points[:, 1]
        if min_val < np.abs(np.max(y_coords) - np.min(y_coords)) < max_val:
            return True
        return False  # didn't pass the filter, then return False

    def height_test(self, points, min_val=10, max_val=100):
        z_coords = points[:, 2]
        if min_val < np.abs(np.max(z_coords) - np.min(z_coords)) < max_val:
            return True
        return False

    def min_height_test(self, points, min_diff=20):
        x_coords = points[:, 0]
        z_coords = points[:, 2]
        if np.min(z_coords) < (self.height_ground + min_diff) or \
                (np.min(x_coords) > 1250 and np.min(z_coords) < (self.height_ground+(min_diff*2))):
            return True
        return False

    def number_of_pts_test(self, points, min_pts=10,max_pts=600):
        if min_pts < points.shape[0] < max_pts:
            return True
        return False

    def cone_shape_egv_test(self, points):
        centroid = np.mean(points, axis=0)
        centered_points = points - centroid
        cov_matrix = np.cov(centered_points.T)
        eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
        idx = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[idx]
        if eigenvalues[0] > eigenvalues[1] * 1.5 and eigenvalues[0] > eigenvalues[2] * 2.5:
            return True
        else:
            return False

    def filter_fov(self):
        # Filter the points that are out of the FOV
        # filter x axis (how far the car can see front and back)
        self.points = self.points[self.points[:, 0] > 0]
        self.points = self.points[self.points[:, 0] < 2000]

        # filter y axis (how far the car can see left and right)
        self.points = self.points[self.points[:, 1] > -500]
        self.points = self.points[self.points[:, 1] < 500]

    def find_ground_o3d(self, pointcloud_np, height_threshold=0):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pointcloud_np)

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=5.0,
            ransac_n=3,
            num_iterations=500
        )

        # Check normal angle
        normal = np.array(plane_model[:3])
        normal /= np.linalg.norm(normal)
        vertical = np.array([0, 0, 1])
        angle = np.arccos(np.clip(np.dot(normal, vertical), -1.0, 1.0))  # angle from vertical in radians
        angle_deg = np.degrees(angle)
        if angle_deg > 45:  # if angle is more than 45 degrees from vertical
            print(f"Plane too far from horizontal: {angle_deg:.2f} degrees. Re-running RANSAC without current inliers.")
            mask = np.ones(len(pointcloud_np), dtype=bool)
            mask[inliers] = False
            new_pointcloud = pointcloud_np[mask]
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(new_pointcloud)
            plane_model, inliers = pcd.segment_plane(
                distance_threshold=5.0,
                ransac_n=3,
                num_iterations=500
            )

        self.plane_model = plane_model  # store for reuse
        [a, b, c, d] = plane_model

        inlier_points = pointcloud_np[inliers]
        self.ground_points = inlier_points

        # Estimate "height_ground" at origin (0, 0) or car base point (you can customize this)
        x_ref, y_ref = 0, 0
        self.height_ground = -(a * x_ref + b * y_ref + d) / c + height_threshold

        print(f"Plane model: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")
        # print(f"Estimated height_ground at (0, 0): {self.height_ground:.2f}")

    def find_ground_skt(self, pointcloud, height_threshold=20, model_option="RANSAC"):
        # Extract the x, y, and z coordinates of the points
        x = pointcloud[:, 0]
        y = pointcloud[:, 1]
        z = pointcloud[:, 2]

        self.ground_model.fit(np.column_stack((x, y)), z)
        self.height_ground = self.ground_model.estimator_.intercept_ + height_threshold
        print(self.ground_model.estimator_.coef_, self.ground_model.estimator_.intercept_)

    def clear_ground(self, pointcloud_np, height_threshold=5):
        # Use the stored ground plane to compute distance from each point to the plane
        if self.ground_points is None:
            print("⚠️ Ground plane not initialized.")
            return

        # Convert numpy to Open3D
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pointcloud_np)

        # Reuse previous plane model (for reuse, you'd need to store it in self)
        # Here we recompute it for simplicity
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=5.0,
            ransac_n=3,
            num_iterations=500
        )
        [a, b, c, d] = plane_model

        # Compute signed distances to the plane
        distances = np.abs((pointcloud_np @ np.array([a, b, c])) + d) / np.linalg.norm([a, b, c])

        # Separate ground/non-ground
        is_ground = distances < height_threshold
        ground_points = pointcloud_np[is_ground]
        non_ground_points = pointcloud_np[~is_ground]

        self.points = non_ground_points
        self.ground_points = ground_points

        # Visualiztion
        # TODO: I dont know how it works, but it is important to make this work.
        # if len(ground_points):
        #     ground_pcd = o3d.geometry.PointCloud()
        #     ground_pcd.points = o3d.utility.Vector3dVector(ground_points)
        #     ground_pcd.paint_uniform_color([0.0, 1.0, 0.0])
        #     o3d.visualization.draw_geometries([ground_pcd], window_name="Ground Points")


    def filter_clusters(self):
        self.points = np.empty((1, 3))  # initialize the final points list.
        filtered_clusters = []
        vaild_cluster = True
        for cluster_num, cluster in enumerate(self.clusters_list):
            for test_num, filter_test in enumerate(self.filter_tests):
                if not filter_test(cluster):  # if the cluster didn't pass the test then continue to next cluster
                    # print(f" cluster num {cluster_num} failed on test {test_num}")
                    vaild_cluster = False
                    break
            if vaild_cluster:
                # if the cluster passed all the filter tests then add it to the final points list and to the clusters
                self.points = np.concatenate((self.points, cluster))
                filtered_clusters.append(cluster)
            vaild_cluster = True
        self.clusters_list = filtered_clusters

    def points_to_clusters(self):
        start = time.time()
        cluster_labels = self.cluster_model.fit_predict(self.points)
        print("dbscan time: ",time.time()-start)
        # Create a list to hold the points in each cluster
        unique_labels = np.unique(cluster_labels)
        cluster_point_lists = []
        # Iterate over the unique cluster labels
        for label in unique_labels:
            # Find the indices of the points in the current cluster
            cluster_indices = np.where(cluster_labels == label)[0]
            # Get the points in the current cluster
            cluster_points = self.points[cluster_indices]
            # Add the points to the list of cluster points
            cluster_point_lists.append(cluster_points)
        self.clusters_list = cluster_point_lists

    def get_cone_centers(self):
        cone_centers = {}
        for cluster_id, cluster in enumerate(self.clusters_list):
            center = np.mean(cluster, axis=0)
            cone_centers[cluster_id] = center
        self.centers = cone_centers

    @staticmethod
    def zero_timer():
        return time.time()

    def rec_time(self, start_time):
        self.component_time_list.append(time.time() - start_time)

    def run(self, points):
        self.points = points

        # trim fov to relevant roi
        self.filter_fov()

        # find ground points
        start = time.time()
        self.find_ground_o3d(self.points)

        # remove ground points
        self.clear_ground(self.points, height_threshold=0)

        # separation to cluster
        self.points_to_clusters()

        # test each cluster to detect cones
        self.filter_clusters()

        # get the centers of valid clusters
        self.get_cone_centers()
        
        return self.points, self.ground_points, self.clusters_list, self.centers


# Create some main function for testing, for now this is the way, but its bad.

# def main():
#     LF = LidarFilter()
#     for i in range(0, 100):
#         with open(f"/Users/poslevkusie/code/BGRacing/BGR-PM-Gregory/Sensors/Lidar/src/algo/detection/python/test_data/PSM/Frame_{i}_PSM.csv", "r") as f:
#             df = pd.read_csv(f)
#             # Filter points where Probability of False Alarm < 0.001
#             filtered_df = df[df['Probability of False Alarm'] < 0.001]
#             points = filtered_df[['x', 'y', 'z',]].to_numpy()
#             if points.shape[0] == 0:
#                 print(f"No valid points in frame {i}")
#                 continue
#             print(f"Loaded {points.shape[0]} points from frame {i}")
#         LF.run(points)

# if __name__ == "__main__":
#     main()
