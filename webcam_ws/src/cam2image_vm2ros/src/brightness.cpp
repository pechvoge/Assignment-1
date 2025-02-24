#include "brightness.hpp"

Brightness::Brightness(const rclcpp::NodeOptions &options)
    : Node("brightness", options)
{
    initialize();
    parse_parameters();
}

void Brightness::initialize(){
    auto qos = rclcpp::QoS(depth_);
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image", qos, std::bind(&Brightness::avg_brightness, this, std::placeholders::_1));

    light_pub_ = this->create_publisher<std_msgs::msg::Bool>("light", qos);    
}

void Brightness::avg_brightness(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImageConstPtr cvimage_ptr;
    cvimage_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat image = cvimage_ptr->image;

    cv::Vec3b rgb;
    unsigned int red, green, blue;
    float rgb_tot = 0.0;
    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            rgb = image.at<cv::Vec3b>(i, j);
            red = rgb[0];
            green = rgb[1];
            blue = rgb[2];
            rgb_tot += (red + green + blue) / 3;
        }
    }

    float brightness = rgb_tot / (image.rows * image.cols);
    // Added get parameter to ensure latest value brightness threshold value is used
    brightness_threshold = this->get_parameter("brightness_threshold").as_int();
    if (brightness > brightness_threshold)
    {
        is_light_on = true;
    }
    else
    {
        is_light_on = false;
    }
    
    RCLCPP_INFO(get_logger(), "Light is %s", is_light_on ? "on" : "off");
    std_msgs::msg::Bool light_msg;
    light_msg.data = is_light_on;
    light_pub_->publish(light_msg);
}

void Brightness::parse_parameters()
{   
    brightness_threshold = this->declare_parameter("brightness_threshold", 100);
    depth_ = this->declare_parameter("depth", 10);
}
