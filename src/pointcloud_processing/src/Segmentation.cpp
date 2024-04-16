#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

class PointCloudSegmentation : public rclcpp::Node {
public:
    PointCloudSegmentation() : Node("point_cloud_segmentation") {
        subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "voxelized_cloud", 10, std::bind(&PointCloudSegmentation::point_cloud_callback, this, std::placeholders::_1));
        
    // Publishers
    floor_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("floor_cloud", 10);
    non_floor_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("non_floor_cloud", 10);
    height_filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("height_filtered_cloud", 10);

    // Declare parameters
    this->declare_parameter<float>("distance_threshold", 0.01);
    this->declare_parameter<float>("z_min", -10);
    this->declare_parameter<float>("z_max", 1);
    }

private:
    void filterPointCloudByHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float z_min, float z_max) {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min, z_max);
        pass.filter(*cloud);  // This will modify the input cloud to contain only the filtered points
    }

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        //Filter the point cloud by height
        float z_min, z_max;
        this->get_parameter("z_min", z_min);
        this->get_parameter("z_max", z_max);
        this->filterPointCloudByHeight(cloud, z_min, z_max);
        sensor_msgs::msg::PointCloud2 height_filtered_output;
        pcl::toROSMsg(*cloud, height_filtered_output);
        //height_filtered_pub_->publish(height_filtered_output); // Uncomment this line to publish the height filtered point cloud

        // Segment the ground
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);

        float distance_threshold;
        this->get_parameter("distance_threshold", distance_threshold);
        seg.setDistanceThreshold(distance_threshold);
        
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Could not estimate a planar model for the given dataset.");
            return;
        }

        // Extract the inliers (floor points)
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
        extract.filter(*cloud_plane);

        // Extract non-floor points
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_floor(new pcl::PointCloud<pcl::PointXYZ>());
        extract.filter(*cloud_non_floor);

        sensor_msgs::msg::PointCloud2 non_floor_output;
        pcl::toROSMsg(*cloud_non_floor, non_floor_output);
        non_floor_output.header.stamp = msg->header.stamp;
        non_floor_output.header.frame_id = msg->header.frame_id;
        non_floor_pub_->publish(non_floor_output);

        sensor_msgs::msg::PointCloud2 floor_output;
        pcl::toROSMsg(*cloud_plane, floor_output);
        floor_output.header.stamp = msg->header.stamp;
        floor_output.header.frame_id = msg->header.frame_id;
        floor_pub_->publish(floor_output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr floor_pub_, non_floor_pub_, height_filtered_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudSegmentation>());
    rclcpp::shutdown();
    return 0;
}
