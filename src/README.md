## General Instruction to build the ROS2 packages
1. The solution to Q1 is contained within `pointclod_processing` package and solution to Q2 is contained within `realtime_pointcloud` package
2. Paste the two mentioned package in the `src` directory of your ROS2 work space and build the workspace
```
colcon build
```
3. Source the setup file
```
source install/setup.bash
```
## To Launch Solution to Question 1

1. Use the following launch command
```
ros2 launch pointcloud_processing pointcloud_processing.launch.py 
```
2. This should launch the `data_publisher`, `noise_filter_node`, `segmentation_node` and `voxelization_node`
3. The above launch file also launches the rviz with the visualization of all the relevant topics

## To Launch Solution to Question 2

1. Use the following launch command
```
ros2 launch realtime_pointcloud realtime_pointcloud.launch.py  
```
2. This should launch the `rosbag_data_processing_node`, `noise_filter_node` and `voxelization_node`
3. The above launch file also launches the rviz with the visualization of all the relevant topics
