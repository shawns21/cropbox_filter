#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

class CropBoxFilterNode : public rclcpp::Node {
public:
    CropBoxFilterNode() : Node("cropbox_filter_node") {

        //Creating Parameters for the min and max values, giving default names to the pointcloud topics
        this->declare_parameter<std::string>("input_cloud", "input_cloud");
        this->declare_parameter<std::string>("output_cloud", "output_cloud");

        this->declare_parameter("min_x", -1.0);
        this->declare_parameter("min_y", -1.0);
        this->declare_parameter("min_z", -1.0);
        this->declare_parameter("max_x", 1.0);
        this->declare_parameter("max_y", 1.0);
        this->declare_parameter("max_z", 1.0);
       
        std::string input_topic, output_topic;
        this->get_parameter("input_cloud", input_topic);
        this->get_parameter("output_cloud", output_topic);

        this->get_parameter("min_x", min_x_);
        this->get_parameter("min_y", min_y_);
        this->get_parameter("min_z", min_z_);
        this->get_parameter("max_x", max_x_);
        this->get_parameter("max_y", max_y_);
        this->get_parameter("max_z", max_z_);
	
        //Creating subscriber that subscribes to the unfiltered pointcloud and the publisher that publishes the filtered pointcloud
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10, std::bind(&CropBoxFilterNode::pointCloudCallback, this, std::placeholders::_1));
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "CropBoxFilterNode started.");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

	//Convert the Pointcloud msg to PCL format
        pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*msg, *pcl_input);

        //Apply the crop box filter
        pcl::PCLPointCloud2::Ptr pcl_cropped(new pcl::PCLPointCloud2());
        pcl::CropBox<pcl::PCLPointCloud2> crop_filter;
        crop_filter.setInputCloud(pcl_input);
        crop_filter.setMin(Eigen::Vector4f(min_x_, min_y_, min_z_, 1.0));
        crop_filter.setMax(Eigen::Vector4f(max_x_, max_y_, max_z_, 1.0));
	crop_filter.setNegative(true);
        crop_filter.filter(*pcl_cropped);

        //Convert back to ROS message and publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl_conversions::fromPCL(*pcl_cropped, output_msg);
        output_msg.header = msg->header; //Keep the original header
        cloud_pub_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    //Crop box parameters
    float min_x_, min_y_, min_z_;
    float max_x_, max_y_, max_z_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CropBoxFilterNode>());
    rclcpp::shutdown();
    return 0;
}

