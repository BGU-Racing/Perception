from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    fps = LaunchConfiguration("fps")
    frame_id = LaunchConfiguration("frame_id")
    enable_timing_log = LaunchConfiguration("enable_timing_log")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("fps", default_value="15.0"),
        DeclareLaunchArgument("frame_id", default_value="lidar"),
        DeclareLaunchArgument("enable_timing_log", default_value="true"),

        # 1) Reader: NPZ -> /lidar/raw (PointCloud2)
        Node(
            package="perception",
            executable="pcl_recording_reader",
            name="pcl_recording_reader_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "fps": fps,
                "frame_id": frame_id,
            }],
        ),

        # 2) Preprocess: /lidar/raw -> /lidar/processed (PointCloud2)
        Node(
            package="perception",
            executable="pcl_preprocess",
            name="pcl_preprocess_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "in_topic": "/lidar/raw",
                "out_topic": "/lidar/processed",
                "publish_frame_id": frame_id,
                "enable_timing_log": enable_timing_log,
            }],
        ),

        # 3) Detection: /lidar/processed -> /lidar/detections (PoseArray)
        Node(
            package="perception",
            executable="pcl_detection",
            name="pcl_detection_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "in_topic": "/lidar/processed",
                "out_topic": "/lidar/detections",
                "frame_id": frame_id,
            }],
        ),
    ])