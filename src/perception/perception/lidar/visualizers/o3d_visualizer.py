import open3d as o3d
import numpy as np


class O3dVisualizer:
    def __init__(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.vis.get_render_option().background_color = np.asarray([0.1, 0.1, 0.1])
        self.vis.get_render_option().point_size = 2
        grid = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        self.vis.add_geometry(grid)
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)

    def play_animation(self, func):
        self.vis.register_animation_callback(func)
        self.vis.run()
        self.vis.destroy_window()

    def add_marker(self, cord):
        marker = o3d.geometry.TriangleMesh.create_cylinder(radius=0.15, height=0.5)
        marker.translate(np.asarray(cord))
        self.vis.add_geometry(marker)

    def add_geo(self, array):
        self.pcd.points = o3d.utility.Vector3dVector(array)
        self.vis.update_geometry(self.pcd)

    def set_camera(self):
        self.vis.get_view_control().set_front([0, 0, 1])
        self.vis.get_view_control().set_lookat([10, 0, 0])
        self.vis.get_view_control().set_up([0, 1, 0])
        self.vis.get_view_control().set_zoom(0.5)
        self.vis.get_view_control().translate(0,0,1)


if __name__ == "__main__":
    pass
