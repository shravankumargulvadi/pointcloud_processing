#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

class DataPublisher : public rclcpp::Node
{
public:
    DataPublisher();
    ~DataPublisher();
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    rclcpp::TimerBase::SharedPtr timer_;
    void publishPointCloud();
};

DataPublisher::DataPublisher() : Node("data_publisher")
{
    this->declare_parameter<std::string>("pcd_file_path", "/home/swathi/swathi/ros2_ws/src/Data/scans.pcd");
    point_cloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("raw_point_cloud", 10);
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&DataPublisher::publishPointCloud, this));
}

DataPublisher::~DataPublisher() {
}

void DataPublisher::publishPointCloud()
{
    std::string file_path;
    this->get_parameter("pcd_file_path", file_path);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load the PCD file.");
        return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    point_cloud_publisher->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published cloud with %d points", cloud->points.size());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataPublisher>());
    rclcpp::shutdown();
    return 0;
}

