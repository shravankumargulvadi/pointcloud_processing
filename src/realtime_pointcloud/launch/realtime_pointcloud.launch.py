from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
    
def generate_launch_description():
    pkg_dir = get_package_share_directory('realtime_pointcloud')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'realtime_pointcloud_viz.rviz')
     
    return LaunchDescription([
        Node(
            package='realtime_pointcloud',
            executable='rosbag_data_processing_node', 
            name='rosbag_data_processing_node',
            output='screen',
            parameters=[]
        ),
        # 'sor_mean_k': 50 and 'sor_stddev_mul_thresh': 0.5 are optimal values for the used dataset
        Node(
            package='pointcloud_processing',
            executable='noise_filter_node', 
            name='noise_filter_node',
            output='screen',
            parameters=[
                {'sor_mean_k': 50, 
                 'sor_stddev_mul_thresh': 1.0}
            ]
        ),
        # 'leaf_size': 0.15 is the optimal value for the used dataset
        Node(
            package='pointcloud_processing',
            executable='voxelization_node', 
            name='voxelization_node',
            output='screen',
            parameters=[
                {'leaf_size': 0.08} 
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])