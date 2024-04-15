#include <cstring>  
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class PointCloudModifier : public rclcpp::Node
{
public:
    PointCloudModifier()
    : Node("rosbag_data_processing_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("raw_point_cloud", 10);
        
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/tof_camera/xyz", 10, std::bind(&PointCloudModifier::listener_callback, this, std::placeholders::_1));
    }

private:

    void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Create a new PointCloud2 message
        sensor_msgs::msg::PointCloud2 modified_msg;
        modified_msg.header = msg->header;
        modified_msg.height = msg->height;
        modified_msg.width = msg->width;
        modified_msg.is_bigendian = msg->is_bigendian;
        modified_msg.point_step = 12; // Only x, y, z fields
        modified_msg.row_step = modified_msg.point_step * msg->width;
        modified_msg.is_dense = msg->is_dense;

        // Define the new fields (only x, y, z)
        modified_msg.fields.resize(3);
        modified_msg.fields[0].name = "x";
        modified_msg.fields[0].offset = 0;
        modified_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        modified_msg.fields[0].count = 1;

        modified_msg.fields[1].name = "y";
        modified_msg.fields[1].offset = 4;
        modified_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        modified_msg.fields[1].count = 1;

        modified_msg.fields[2].name = "z";
        modified_msg.fields[2].offset = 8;
        modified_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        modified_msg.fields[2].count = 1;

        // Allocate new data vector
        modified_msg.data.resize(modified_msg.row_step * msg->height);

        // Copy only the x, y, z data
        for (size_t i = 0; i < msg->width * msg->height; ++i) {
            memcpy(&modified_msg.data[i * modified_msg.point_step], 
                   &msg->data[i * msg->point_step],
                   modified_msg.point_step);
        }
        // resize x,y and z
        sensor_msgs::PointCloud2Iterator<float> iter_x(modified_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(modified_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(modified_msg, "z");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            *iter_x /= 100;
            *iter_y /= 100;
            *iter_z /= 100;
        }
        

        // Publish the modified message
        publisher_->publish(modified_msg);

    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudModifier>());
    rclcpp::shutdown();
    return 0;
}
