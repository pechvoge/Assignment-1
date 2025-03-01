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
    CoG_sub_ = this->create_subscription<geometry_msgs::msg::Point>("CoG", qos, std::bind(&Sequence_generator_node::updateDesiredPoint, this, std::placeholders::_1));
    camera_location_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/output/camera_position", qos, std::bind(&Sequence_generator_node::updateCameraCoords, this, std::placeholders::_1));
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/input/twist", qos);

    random_twist.linear.x = 0.0;
    random_twist.linear.y = 0.0;
    random_twist.linear.z = 0.0;
    random_twist.angular.x = 0.0;
    random_twist.angular.y = 0.0;
    random_twist.angular.z = 0.0;
    
    init_time = get_clock()->now();

    pub_timer_ =(assignment_selecter_) ? this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000*dt_)),
        std::bind(&Sequence_generator_node::sequenceController, this))
    :
        this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(500*dt_)),
        std::bind(&Sequence_generator_node::publisherCallback, this));
}

void Sequence_generator_node::absolute(float &delta_x)
{
    if (delta_x < 0)
    {
        delta_x = -delta_x;
    } // No else statement needed due to pass by reference
}

void Sequence_generator_node::updateCameraCoords(const geometry_msgs::msg::PointStamped::SharedPtr camPosMsg)
{
    pix_x0_ = camPosMsg->point.x;
    pix_y0_ = camPosMsg->point.y;
    //RCLCPP_INFO(get_logger(), "Camera x: %f, Camera y: %f", pix_x0_, pix_y0_);
}

void Sequence_generator_node::updateDesiredPoint(const geometry_msgs::msg::Point::SharedPtr CoGMsg)
{
    desired_point_x = CoGMsg->x;
    desired_point_y = CoGMsg->y;
    //RCLCPP_INFO(get_logger(), "CoG x: %f, CoG y: %f", desired_point_x, desired_point_y);
}

void Sequence_generator_node::sequenceController()
{
    float pix_diff_x = pix_x0_ - desired_point_x;
    float pix_diff_y = pix_y0_ - desired_point_y;
    RCLCPP_INFO(this->get_logger(), "Pix diff x: %f, Pix diff y: %f", pix_diff_x, pix_diff_y);
    absolute(pix_diff_y);
    RCLCPP_INFO(this->get_logger(), "Absolute pix diff y: %f", pix_diff_y);
    
    scaling_factor_theta_ = this->get_parameter("scaling_factor_theta").as_double();
    scaling_factor_x_ = this->get_parameter("scaling_factor_x").as_double();
    random_twist.angular.z = scaling_factor_theta_*pix_diff_x/dt_;
    if ((pix_diff_y < 180 && pix_diff_y > 60) || (pix_diff_y < 10))
    {
        random_twist.linear.x = -scaling_factor_x_/dt_;
    } else 
    {
        random_twist.linear.x = scaling_factor_x_/dt_;
    }
    
    twist_pub_->publish(random_twist);
    RCLCPP_INFO(this->get_logger(), "Publishing twist: %f, %f", random_twist.linear.x, random_twist.angular.z);
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
    assignment_selecter_ = this->declare_parameter("assignment_selecter", true);
    dt_ = this->declare_parameter("dt", 1.0);
    scaling_factor_theta_ = this->declare_parameter("scaling_factor_theta", 1e-2);
    scaling_factor_x_ = this->declare_parameter("scaling_factor_x", 3e-1);
}
