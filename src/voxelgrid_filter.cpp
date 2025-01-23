#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class VoxelFilterNode : public rclcpp::Node {

public:
    VoxelFilterNode() : Node("voxelgrid_filter_node"){

        this->declare_parameter<std::string>("input_cloud", "input_cloud");
        this->declare_parameter<std::string>("output_cloud", "output_cloud");

        this->declare_parameter("leaf_size", 0.1);

        std::string input_topic, output_topic;

        this->get_parameter("input_cloud", input_topic);
        this->get_parameter("output_cloud", output_topic);

        this->get_parameter("leaf_size", leaf_size_);

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10, std::bind(&VoxelFilterNode::pointCloudCallback, this, std::placeholders::_1));

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "VoxelFilterNode started");

    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

        pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*msg, *pcl_input);

        pcl::PCLPointCloud2::Ptr pcl_filtered(new pcl::PCLPointCloud2());
        pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
        voxel_filter.setInputCloud(pcl_input);
        voxel_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        voxel_filter.filter(*pcl_filtered);

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl_conversions::fromPCL(*pcl_filtered, output_msg);
        output_msg.header = msg->header; //Keep the original header
        cloud_pub_->publish(output_msg);

    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    float leaf_size_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelFilterNode>());
    rclcpp::shutdown();
    return 0;
}
