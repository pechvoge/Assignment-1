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

    brightness = rgb_tot / (image.rows * image.cols);
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
    //light_pub_->publish(std::make_shared<std_msgs::msg::Bool>(is_light_on)); // pointer?
}

void Brightness::parse_parameters()
{   
    brightness = this->declare_parameter("brightness", 0.0);
    brightness_threshold = this->declare_parameter("brightness_threshold", 100);
    is_light_on = this->declare_parameter("is_light_on", false);
    input_topic_ = this->declare_parameter("input_topic", "image");
    output_topic_ = this->declare_parameter("output_topic", "light");
    depth_ = this->declare_parameter("depth", 10);
}
