#include "sequence_generator_pkg/sequence_generator_node.hpp"

Sequence_generator_node::Sequence_generator_node(const rclcpp::NodeOptions &options)
    : Node("sequence_generator_node", options)
{
    parse_parameters();
    initialize();
}

void Sequence_generator_node::initialize()
{
    auto qos = rclcpp::QoS(depth_);
    camera_location_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/output/camera_position", qos, std::bind(&Sequence_generator_node::updateCameraCoords, this, std::placeholders::_1));
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/input/twist", qos);

    random_twist.linear.x = 0.0;
    random_twist.linear.y = 0.0;
    random_twist.linear.z = 0.0;
    random_twist.angular.x = 0.0;
    random_twist.angular.y = 0.0;
    random_twist.angular.z = 0.0;
    
    init_time = get_clock()->now();

    pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(500.0)),
        std::bind(&Sequence_generator_node::publisherCallback, this));
}

void Sequence_generator_node::updateCameraCoords(const geometry_msgs::msg::PointStamped::SharedPtr camPosMsg)
{
    pix_x0_ = camPosMsg->point.x;
    pix_y0_ = camPosMsg->point.y;
    //RCLCPP_INFO(get_logger(), "Camera x: %f, Camera y: %f", pix_x0_, pix_y0_);
}

void Sequence_generator_node::publisherCallback()
{
    auto current_time = get_clock()->now();
    float time_diff = (current_time - init_time).seconds();
    RCLCPP_INFO(this->get_logger(), "Time difference: %f", time_diff);
    if (time_diff < 5)
    {
        random_twist.linear.x = twist_strength_;
    } else if (time_diff < 10)
    {
        random_twist.linear.x = -twist_strength_;
    } else if (time_diff < 15)
    {
        random_twist.linear.x = 0.0;
        random_twist.angular.z = twist_strength_;
    } else if (time_diff < 20)
    {
        random_twist.angular.z = -twist_strength_;
    } else 
    {
        random_twist.angular.z = 0.0;
        init_time += rclcpp::Duration::from_seconds(20);
    }
    twist_pub_->publish(random_twist);
    RCLCPP_INFO(this->get_logger(), "Publishing twist: %f, %f", random_twist.linear.x, random_twist.angular.z);
}

void Sequence_generator_node::parse_parameters()
{
    depth_ = this->declare_parameter("depth", 10);
    twist_strength_ = this->declare_parameter("twist_strength", 1.0);
}
