#include "sequence_generator_pkg/sequence_generator_node.hpp"

Sequence_generator_node::Sequence_generator_node(const rclcpp::NodeOptions &options)
    : Node("sequence_generator_node", options)
{
    parse_parameters();
    initialize();

    Sequence_generator();
    init_time_ = this->now();
    pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(dt*1000)),
        std::bind(&Sequence_generator_node::publisherCallback, this));
}

void Sequence_generator_node::absolute(float &delta_x)
{
    if (delta_x < 0)
    {
        delta_x = -delta_x;
    } // No else statement needed due to pass by reference
}

float Sequence_generator_node::power(float base, float exponent)
{
    for (int i = 0; i < exponent; i++)
    {
        base *= base;
    }
    return base;
}

void Sequence_generator_node::initialize()
{
    auto qos = rclcpp::QoS(depth_);
    // image_sub_ = this->create_subscription<sensor_msgs::msg::Image<("image", qos, std::bind(&Object_detection_node::CoG_determiner, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/input/twist", qos);

    desired_point.x = 180;
    desired_point.y = 0;
    desired_point.z = 0;

    desired_point_pointer->x = desired_point.x;
    desired_point_pointer->y = desired_point.y;
    desired_point_pointer->z = desired_point.z;

    zero_twist.linear.x = 0;
    zero_twist.linear.y = 0;
    zero_twist.linear.z = 0;
    zero_twist.angular.x = 0;
    zero_twist.angular.y = 0;
    zero_twist.angular.z = 0;
}

void Sequence_generator_node::desiredPose()
{
    float delta_pix_x = pix_x0_ - desired_point_pointer->x;
    float delta_pix_y = pix_y0_ - desired_point_pointer->y;
    absolute(delta_pix_y);
    // define as conversion factor between window size
    theta_zf_ = 2.0 / 230.0 * delta_pix_x; // width of window / 2 - width of sub image / 2= 320 - 90 = 230 and max angle is 2
    xf_ = -5.0 / 180.0 * delta_pix_y;      // delta y at the top of the window = 180 and zoom is -5
    RCLCPP_INFO(this->get_logger(), "Desired pose: theta = %f, x = %f", theta_zf_, xf_);
}

std::array<float, 4> Sequence_generator_node::getCoefficients(float final_time, float ini_q, float final_q)
{
    float a0 = ini_q;
    float a1 = 0;
    float a2 = 3 * (final_q - ini_q) / power(final_time, 2);
    float a3 = -2 * (final_q - ini_q) / power(final_time, 3);
    return {a0, a1, a2, a3};
}

void Sequence_generator_node::Sequence_generator()
{
    desiredPose();
    std::array<float, 4> a = getCoefficients(final_time, theta_z0_, theta_zf_);
    std::array<float, 4> b = getCoefficients(final_time, x0_, xf_);
    geometry_msgs::msg::Twist twist;

    float t = 0.0;
    while(t <= 3.0*final_time)
    {
    twist.angular.z = a[1] + 2.0 * a[2] * t + 3.0 * a[3] * power(t, 2);
    twist.linear.x = b[1] + 2.0 * b[2] * t + 3.0 * b[3] * power(t, 2);
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;

    RCLCPP_INFO(this->get_logger(), "Twist: angular.z=%f, linear.x=%f", twist.angular.z, twist.linear.x);
    twist_buffer.push_back(twist);
    t += dt;
    RCLCPP_INFO(this->get_logger(), "Time: %f", t);
    }
    
}

void Sequence_generator_node::publisherCallback()
{
    if (twist_buffer.empty())
    {
        twist_pub_->publish(zero_twist);
        RCLCPP_INFO(this->get_logger(), "Publishing zero twist");
        return;
    }
    else
    {
        geometry_msgs::msg::Twist twist = twist_buffer.front();
        twist_buffer.erase(twist_buffer.begin());
        twist_pub_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Publishing twist: angular.z=%f, linear.x=%f", twist.angular.z, twist.linear.x);
    }
}

void Sequence_generator_node::parse_parameters()
{
    pub_freq_ = this->declare_parameter("publisher_frequency", 30.0);
    depth_ = this->declare_parameter("depth", 10);
}
