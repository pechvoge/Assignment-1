#include "object_detection_pkg/object_detection_node.hpp"

Object_detection_node::Object_detection_node(const rclcpp::NodeOptions &options)
    : Node("object_detection_node", options)
{
    parse_parameters();
    initialize();
}

void Object_detection_node::initialize(){
    auto qos = rclcpp::QoS(depth_);
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image", qos, std::bind(&Object_detection_node::CoG_determiner, this, std::placeholders::_1));

    CoG_pub_ = this->create_publisher<geometry_msgs::msg::Point>("CoG", qos);    
}

void Object_detection_node::CoG_determiner(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImageConstPtr cvimage_ptr;
    cvimage_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat image = cvimage_ptr->image;
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_RGB2GRAY);
    gray_threshold_ = this->get_parameter("gray_threshold").as_int();
    cv::Mat thresholded_image;
    cv::threshold(gray_image, thresholded_image, gray_threshold_, 255, cv::THRESH_BINARY);
    cv::Moments mom = cv::moments(thresholded_image, true);
    geometry_msgs::msg::Point CoG;
    CoG.x = mom.m10 / mom.m00;
    CoG.y = mom.m01 / mom.m00;
    CoG.z = 0;
    CoG_pub_->publish(CoG);
    RCLCPP_INFO(get_logger(), "CoG is at (%f, %f)", CoG.x, CoG.y);
    cv::imshow("object", thresholded_image);
    cv::waitKey(1);
}

void Object_detection_node::parse_parameters()
{   
    gray_threshold_ = this->declare_parameter("gray_threshold", 100);
    depth_ = this->declare_parameter("depth", 10);
}