
// ROS Headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
//#include "std_msgs/msg/Bool.hpp"
// PCL Headers
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <boost/shared_ptr.hpp>

class Voxelization : public rclcpp::Node
{
public:
    Voxelization();
    ~Voxelization();
    float leaf_size;// Voxel grid leaf size
    pcl::VoxelGrid<pcl::PointXYZ> grid;
private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxelized_cloud_publisher; 
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_subscriber;

};  

Voxelization::Voxelization(): Node("voxelization")
{
    // Create a publisher
    voxelized_cloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxelized_cloud", 10);
    // Create a subscriber
    filtered_cloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10, std::bind(&Voxelization::callback, this, std::placeholders::_1));
    // make leaf_size a parameter
    this->declare_parameter<float>("leaf_size", 0.1);
}

Voxelization::~Voxelization()
{
}

void Voxelization::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{ 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxelized(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *cloud_filtered); 
    grid.setInputCloud(cloud_filtered);
    this->get_parameter("leaf_size", leaf_size);
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.filter(*cloud_voxelized);
    // Publish the data
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_voxelized, output);
    output.header.frame_id = "base_link";
    voxelized_cloud_publisher->publish(output);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Voxelization>());
    rclcpp::shutdown();
    return 0;
}