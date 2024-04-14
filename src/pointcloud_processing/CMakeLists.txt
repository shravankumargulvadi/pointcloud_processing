cmake_minimum_required(VERSION 3.5)
project(pointcloud_processing)

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
find_package(rviz2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io segmentation)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

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

add_executable(voxelization_node src/Voxelization.cpp)
ament_target_dependencies(voxelization_node
  rclcpp
  std_msgs
  sensor_msgs
  PCL
)
target_link_libraries(voxelization_node ${PCL_LIBRARIES})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

install(TARGETS
  voxelization_node 
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(data_publisher_node src/DataPublisher.cpp)
ament_target_dependencies(data_publisher_node
  rclcpp
  sensor_msgs
  PCL
)
target_link_libraries(data_publisher_node ${PCL_LIBRARIES})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

install(TARGETS
  data_publisher_node 
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(noise_filter_node src/NoiseFilter.cpp)
ament_target_dependencies(noise_filter_node
  rclcpp
  sensor_msgs
  PCL
)
target_link_libraries(noise_filter_node ${PCL_LIBRARIES})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

install(TARGETS
  noise_filter_node 
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(segmentation_node src/Segmentation.cpp)
ament_target_dependencies(segmentation_node
  rclcpp
  sensor_msgs
  PCL
)
target_link_libraries(segmentation_node ${PCL_LIBRARIES})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

install(TARGETS
  segmentation_node 
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
