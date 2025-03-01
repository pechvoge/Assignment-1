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
    // Convert ros image to opencv image
    cv_bridge::CvImageConstPtr cvimage_ptr;
    cvimage_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat image = cvimage_ptr->image;

    // Convert to grayscale
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_RGB2GRAY);

    // Threshold grayscaled image
    gray_threshold_ = this->get_parameter("gray_threshold").as_int(); // Ensure always the newest value is used
    cv::Mat thresholded_image;
    cv::threshold(gray_image, thresholded_image, gray_threshold_, 255, cv::THRESH_BINARY);

    // Use of image moments to determine the center of gravity
    cv::Moments mom = cv::moments(thresholded_image, true);
    geometry_msgs::msg::Point CoG;
    CoG.x = mom.m10 / mom.m00;
    CoG.y = mom.m01 / mom.m00;
    CoG.z = 0;

    // Publish the center of gravity
    CoG_pub_->publish(CoG);

    // For ease of debugging, print the center of gravity to the console and display the thresholded image
    RCLCPP_INFO(get_logger(), "CoG is at (%f, %f)", CoG.x, CoG.y);
    cv::imshow("object", thresholded_image);
    cv::waitKey(1);
}

void Object_detection_node::parse_parameters()
{   
    gray_threshold_ = this->declare_parameter("gray_threshold", 240);
    depth_ = this->declare_parameter("depth", 10);
}