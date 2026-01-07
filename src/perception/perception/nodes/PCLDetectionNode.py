import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray

from perception.utils.pc2_npy import pointcloud2_to_numpy, centers_to_pose
from perception.lidar.algo.detection.python.filters_algo import LidarFilter


class PCLDetectionNode(Node):
    def __init__(self):
        super().__init__("pcl_detection_node")

        self.declare_parameter("in_topic", "/lidar/processed")
        self.declare_parameter("out_topic", "/lidar/detections")
        self.declare_parameter("frame_id", "lidar")

        self.in_topic = self.get_parameter("in_topic").value
        self.out_topic = self.get_parameter("out_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.sub = self.create_subscription(PointCloud2, self.in_topic, self.call_back, qos_profile_sensor_data)
        self.pub = self.create_publisher(PoseArray, self.out_topic, 10)

        self.lidar_filter = LidarFilter()
        self.get_logger().info(f"Detection: {self.in_topic} -> {self.out_topic}")

    def call_back(self, msg: PointCloud2):
        try:
            points = pointcloud2_to_numpy(msg)  
        except Exception as e:
            self.get_logger().error(f"Failed to decode PointCloud2: {e}")
            return

        # Empty input cloud -> publish empty detections
        if points.shape[0] == 0:
            self.get_logger().warn("Empty pointcloud frame -> publishing empty detections", throttle_duration_sec=2.0)
            pa = centers_to_pose({}, msg.header, self.frame_id)
            self.pub.publish(pa)
            return

        # Run your clustering pipeline
        self.lidar_filter.points = points
        self.lidar_filter.points_to_clusters()
        self.lidar_filter.filter_clusters()
        self.lidar_filter.get_cone_centers()

        pa = centers_to_pose(self.lidar_filter.centers, msg.header, self.frame_id)
        self.pub.publish(pa)


def main():
    rclpy.init()
    node = PCLDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()