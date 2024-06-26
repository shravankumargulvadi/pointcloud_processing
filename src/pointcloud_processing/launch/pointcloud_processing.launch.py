from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
    
def generate_launch_description():
    pkg_dir = get_package_share_directory('pointcloud_processing')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'pointcloud_viz.rviz')
     
    return LaunchDescription([
        Node(
            package='pointcloud_processing',
            executable='data_publisher_node', 
            name='data_publisher_node',
            output='screen',
            parameters=[]
        ),
        Node(
            package='pointcloud_processing',
            executable='noise_filter_node', 
            name='noise_filter_node',
            output='screen',
            parameters=[
                {'sor_mean_k': 50, # 'sor_mean_k': 50 and 'sor_stddev_mul_thresh': 0.5 are optimal values for the used dataset
                 'sor_stddev_mul_thresh': 0.5}
            ]
        ),
        Node(
            package='pointcloud_processing',
            executable='voxelization_node', 
            name='voxelization_node',
            output='screen',
            parameters=[
                {'leaf_size': 0.1} # 'leaf_size': 0.15 is the optimal value for the used dataset
            ]
        ),
        Node(
            package='pointcloud_processing',
            executable='segmentation_node', 
            name='segmentation_node',
            output='screen',
            parameters=[
                {'distance_threshold': 0.1} # 'distance_threshold': 0.01 is the optimal value for the used dataset
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