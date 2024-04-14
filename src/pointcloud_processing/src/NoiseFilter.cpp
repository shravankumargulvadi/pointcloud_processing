#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

class OutlierRemovalNode : public rclcpp::Node {
public:
    OutlierRemovalNode() : Node("noise_filter_node") {
        subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "raw_point_cloud", 10, std::bind(&OutlierRemovalNode::callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
        // declare SOR parameters
        this->declare_parameter<int16_t>("sor_mean_k", 50); // number of neighbors to analyze for each point
        this->declare_parameter<float>("sor_stddev_mul_thresh", 1.0); // standard deviation multiplier threshold
    }

    ~OutlierRemovalNode() {
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::fromROSMsg(*msg, *cloud);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);

        int16_t sor_mean_k;
        float sor_stddev_mul_thresh;
        this->get_parameter("sor_mean_k", sor_mean_k);
        this->get_parameter("sor_stddev_mul_thresh", sor_stddev_mul_thresh);

        sor.setMeanK(sor_mean_k);
        sor.setStddevMulThresh(sor_stddev_mul_thresh);
        sor.filter(*cloud_filtered);

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header.stamp = this->get_clock()->now();
        output.header.frame_id = "base_link";
        publisher_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OutlierRemovalNode>());
    rclcpp::shutdown();
    return 0;
}