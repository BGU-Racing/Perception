import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2

import numpy as np

from perception.lidar.readers.python.record_reader import NPZFrameReader
from perception.utils.pc2_npy import make_fields, numpy_to_pointcloud2


class PCLRecordingReader(Node):
    def __init__(self):
        super().__init__("pcl_recording_reader_node")

        self.declare_parameter("fps", 15.0)
        self.declare_parameter("frame_id", "lidar")

        self.fps = float(self.get_parameter("fps").value)
        self.default_frame_id = self.get_parameter("frame_id").value

        self.reader = NPZFrameReader()

        self.pub = self.create_publisher(PointCloud2, "/lidar/raw", qos_profile_sensor_data)
        self.fields, self.point_step = make_fields()

        period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(f"Publishing /lidar/raw at ~{self.fps} Hz using NPZFrameReader()", throttle_duration_sec=1.0)

    def on_timer(self):
        try:
            arr = self.reader.read() 
        except StopIteration:
            self.get_logger().info("Frame sequence finished.")
            return
        except Exception as e:
            self.get_logger().error(f"Reader error: {e}")
            return

        # arr has fields: x,y,z, Epoch time (usec), Frame, ...
        # xyz = np.vstack([arr["x"], arr["y"], arr["z"]]).T.astype(np.float32) - Understand which option is faster
        xyz = np.empty((arr.shape[0], 3), dtype=np.float32)
        xyz[:, 0] = arr["x"]
        xyz[:, 1] = arr["y"]
        xyz[:, 2] = arr["z"]

        header = Header()
        header.frame_id = self.default_frame_id

        # timestamp from your file if present
        if "Epoch time (usec)" in arr.dtype.names:
            epoch_us = int(arr["Epoch time (usec)"][0])
            sec = epoch_us // 1_000_000
            nsec = (epoch_us % 1_000_000) * 1000
            header.stamp.sec = int(sec)
            header.stamp.nanosec = int(nsec)

        msg = numpy_to_pointcloud2(xyz, header, self.fields, self.point_step)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PCLRecordingReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()