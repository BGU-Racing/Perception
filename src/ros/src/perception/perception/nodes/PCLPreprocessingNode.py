import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from perception.utils.pc2_npy import make_fields, numpy_to_pointcloud2, pointcloud2_to_numpy
from perception.lidar.algo.calibration.python.auto_calibration import auto_calibrate_point_cloud
from perception.lidar.algo.detection.python.filters_algo import LidarFilter


class PCLPreprocessNode(Node):
    def __init__(self):
        super().__init__("pcl_preprocess_node")

        self.declare_parameter("in_topic", "/lidar/raw")
        self.declare_parameter("out_topic", "/lidar/processed")
        self.declare_parameter("publish_frame_id", "lidar")
        self.declare_parameter("enable_timing_log", True)

        self.in_topic = self.get_parameter("in_topic").value
        self.out_topic = self.get_parameter("out_topic").value
        self.publish_frame_id = self.get_parameter("publish_frame_id").value
        self.enable_timing_log = bool(self.get_parameter("enable_timing_log").value)

        self.sub = self.create_subscription(PointCloud2, self.in_topic, self.call_back, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, self.out_topic, qos_profile_sensor_data)

        self.fields, self.point_step = make_fields()

        self.lidar_filter = LidarFilter()

        self.is_calibrated = False
        self.rot_mat = None
        self.T = None

        self.get_logger().info(f"Preprocess: {self.in_topic} -> {self.out_topic}")

    def call_back(self, msg: PointCloud2):
        t0 = time.time()

        # 1) pc2 -> numpy (N,3)
        pcd_frame = pointcloud2_to_numpy(msg)  # float32

        # 2) keep a copy for calibration input
        orig_frame = pcd_frame

        # 3) set points + fov
        self.lidar_filter.points = pcd_frame
        self.lidar_filter.filter_fov()
        t1 = time.time()

        # 4) find ground (o3d)
        self.lidar_filter.find_ground_o3d(self.lidar_filter.points)
        t2 = time.time()

        # 5) plane coefficients
        a, b, c, d = self.lidar_filter.plane_model

        # 6) one-time auto-calibration 
        if not self.is_calibrated:
            rot, trans = auto_calibrate_point_cloud(orig_frame, a, b, d)
            self.rot_mat = np.asarray(rot, dtype=np.float32)
            self.T = np.asarray(trans, dtype=np.float32).reshape(1, 3)
            self.is_calibrated = True
        t3 = time.time()

        # 7) clear ground
        #    NOTE: your clear_ground sets self.lidar_filter.points = non_ground_points - gpt remark
        self.lidar_filter.clear_ground(self.lidar_filter.points)
        non_ground = self.lidar_filter.points
        t4 = time.time()

        # 8) rotate + translate
        processed = (self.rot_mat @ non_ground.T).T + self.T
        t5 = time.time()

        # 9) publish as PointCloud2
        header = msg.header
        header.frame_id = self.publish_frame_id

        out_msg = numpy_to_pointcloud2(processed.astype(np.float32), header, self.fields, self.point_step)
        self.pub.publish(out_msg)

        if self.enable_timing_log:
            self.get_logger().info(
                f"preprocess ms: pc2={(t1-t0)*1000:.1f} "
                f"plane={(t2-t1)*1000:.1f} "
                f"calib={(t3-t2)*1000:.1f} "
                f"clear_ground={(t4-t3)*1000:.1f} "
                f"rt={(t5-t4)*1000:.1f} total={(t5-t0)*1000:.1f} "
                f"pts_in={orig_frame.shape[0]} pts_out={processed.shape[0]}"
            )

def main():
    rclpy.init()
    node = PCLPreprocessNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()