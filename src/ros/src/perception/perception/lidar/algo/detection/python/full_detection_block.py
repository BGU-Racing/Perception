import time
import numpy as np

from perception.lidar.algo.calibration.python.auto_calibration import auto_calibrate_point_cloud

from perception.lidar.readers.python.record_reader import NPZFrameReader #.device_reader import DeviceReader

from perception.lidar.algo.detection.python.filters_algo import LidarFilter

from perception.lidar.visualizers.o3d_visualizer import O3dVisualizer

class LidarModule:
    def __init__(self):
        self.lidar_reader = NPZFrameReader() # DeviceReader()
        self.lidar_filter = LidarFilter()
        self.rot_mat = None
        self.trans_vec = None
        self.vis = None
        self.R = None
        self.T = None
        self.is_calibrated = False
        
    def scan_detect(self):
        pcd_frame = None
        while pcd_frame is None:
            pcd_frame = self.lidar_reader.read() # callback()
        # start = time.time()
        pcd_frame = np.vstack([pcd_frame['x'], pcd_frame['y'], pcd_frame['z']]).T

        self.lidar_filter.points = pcd_frame
        self.lidar_filter.filter_fov()
        start = time.time()
        self.lidar_filter.find_ground_o3d(self.lidar_filter.points)
        print("filter ground ", time.time() - start)

        # calculate rotation matrix and rotate pcd ,then add translation
        a, b, c, d = self.lidar_filter.plane_model

        if not self.is_calibrated:
            rot, trans = auto_calibrate_point_cloud(pcd_frame, a, b, d)
            self.rot_mat = rot
            self.T = trans
            self.is_calibrated = True
            print("auto calibrate: ", time.time() - start)

        self.lidar_filter.clear_ground(self.lidar_filter.points)
        pcd_frame = np.matmul(self.rot_mat, self.lidar_filter.points.T).T + self.T
        print("filter ground and fix rotation: ", time.time() - start)
        # filtering and clustering of point to get detections
        _, _, _, detections = self.lidar_filter.run(pcd_frame)
        # print(time.time()-start)
        return detections

    # TODO: Understand which of visualizations we want to use, here or there, they have different parametrs, also the scan_detect_visualize() is like scan_detect(), but with visualization, imho move to other place.
    def show_detections(self): 
        self.vis = O3dVisualizer()
        self.vis.vis.get_view_control().set_front([1, 0, 0])
        self.vis.vis.get_view_control().set_lookat([10, 0, 0])
        self.vis.vis.get_view_control().set_up([0, 0, 1])
        self.vis.vis.get_view_control().set_zoom(10)
        self.vis.play_animation(self.scan_detect_visualize)

    def scan_detect_visualize(self, vis):
        pcd_frame = None
        while pcd_frame is None:
            pcd_frame = self.lidar_reader.read() # callback()
        pcd_frame = np.vstack([pcd_frame['x'], pcd_frame['y'], pcd_frame['z']]).T
        self.lidar_filter.points = pcd_frame
        self.lidar_filter.filter_fov()
        self.lidar_filter.find_ground(self.lidar_filter.points)

        # calculate rotation matrix and rotate pcd ,then add translation
        a, b = self.lidar_filter.ground_model.estimator_.coef_
        d = self.lidar_filter.ground_model.estimator_.intercept_
        rot, trans = auto_calibrate_point_cloud(pcd_frame, a, b, d)
        self.lidar_filter.clear_ground(self.lidar_filter.points)
        pcd_frame = np.matmul(rot, self.lidar_filter.points.T).T + trans

        # filtering and clustering of point to get detections
        _, _, _, detections = self.lidar_filter.run(pcd_frame)

        # add geometries
        self.vis.add_geo(pcd_frame / 100)

        for det_id in detections:
            pred = detections[det_id].reshape(-1, 1) / 100
            self.vis.add_marker(pred)
        return True


if __name__ == '__main__':
    lidar_mod = LidarModule()
    while True:
        start = time.time()
        lidar_mod.scan_detect()
        print(time.time() - start)