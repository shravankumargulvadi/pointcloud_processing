cmake_minimum_required(VERSION 3.5)
project(realtime_pointcloud)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz2 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



add_executable(rosbag_data_processing_node src/RosbagDataProcessing.cpp)
ament_target_dependencies(rosbag_data_processing_node
  rclcpp
  sensor_msgs
  PCL
  tf2_ros
  geometry_msgs
)
target_link_libraries(rosbag_data_processing_node ${PCL_LIBRARIES})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

install(TARGETS
  rosbag_data_processing_node 
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
