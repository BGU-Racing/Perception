import open3d as o3d
import numpy as np
import time

def auto_calibrate_point_cloud(point_array, a, b, d):
    """
    Calibrates a point cloud from a point_array file by aligning the floor plane with the XY plane.

    Parameters:
        point_array: file containing the point cloud data with three columns (x, y, z).

    Returns:
        tuple: Rotation matrix (R) and translation vector (T) applied to the point cloud.
    """
    start = time.time() 
    # Convert the data into an Open3D point cloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_array)

    c = 1

    # Calculate the normal vector and determine rotation to align with the z-axis
    normal_vector = np.array([a, b, c])
    z_axis = np.array([0, 0, 1])
    axis = np.cross(normal_vector, z_axis)
    angle = np.arccos(np.dot(normal_vector, z_axis) / (np.linalg.norm(normal_vector) * np.linalg.norm(z_axis)))
    axis /= np.linalg.norm(axis)

    # Compute rotation matrix
    R = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)
    print("time to compute rotation matrix: ", time.time() - start)
    # pcd.rotate(R)

    # Compute translation vector to align plane to z = 0
    T = -d * np.array([a, b, c]) / (a ** 2 + b ** 2 + c ** 2)
    # pcd.translate(T)

    return R, T

# ************************** Example of use: ********************************
#
# import Auto_Calibration
# R, T = Auto_Calibration.Auto_calibrate_point_cloud("csv_file")
# print("Rotation Matrix (R):\n", R)
# print("Translation Vector (T):\n", T)



