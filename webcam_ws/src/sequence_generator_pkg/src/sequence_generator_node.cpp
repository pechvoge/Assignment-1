#include "sequence_generator_pkg/sequence_generator_node.hpp"

Sequence_generator_node::Sequence_generator_node(const rclcpp::NodeOptions &options)
    : Node("sequence_generator_node", options)
{
    parse_parameters();
    initialize();

    compute_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(10000.0 / pub_freq_)),
        std::bind(&Sequence_generator_node::Sequence_generator, this));
    
    pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / pub_freq_)),
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

    desired_point.x = 90;
    desired_point.y = 0;
    desired_point.z = 0;
    zero_twist.linear.x = 0;
    zero_twist.linear.y = 0;
    zero_twist.linear.z = 0;
    zero_twist.angular.x = 0;
    zero_twist.angular.y = 0;
    zero_twist.angular.z = 0;
}

std::array<float, 2> Sequence_generator_node::desiredPose()
{
    float delta_pix_x = pix_x0_ - desired_point_pointer->x;
    float delta_pix_y = pix_y0_ - desired_point_pointer->y;
    absolute(delta_pix_y);
    // define as conversion factor between window size
    theta_zf_ = 2 / 230 * delta_pix_x; // width of window / 2 - width of sub image / 2= 320 - 90 = 230 and max angle is 2
    xf_ = -5 / 180 * delta_pix_y;      // delta y at the top of the window = 180 and zoom is -5
    return {theta_zf_, xf_};
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
    std::array<float, 2> pose = desiredPose();
    std::array<float, 4> a = getCoefficients(final_time, theta_z0_, pose[0]);
    std::array<float, 4> b = getCoefficients(final_time, x0_, pose[1]);

    int t = 0;
    geometry_msgs::msg::Twist twist;
    for (size_t i = 0; i < buffer_size; i++)
    {
        t += dt;
        twist.angular.z = a[0] + a[1] * t + a[2] * power(t, 2) + a[3] * power(t, 3);
        twist.linear.x = b[0] + b[1] * t + b[2] * power(t, 2) + b[3] * power(t, 3);
        twist_buffer.push_back(twist);
    }
}

void Sequence_generator_node::publisherCallback()
{
    if (twist_buffer.empty())
    {
        twist_pub_->publish(zero_twist);
    }
    else
    {
        twist_pub_->publish(twist_buffer.back());
        twist_buffer.pop_back();
    }
}

void Sequence_generator_node::parse_parameters()
{
    pub_freq_ = this->declare_parameter("publisher_frequency", 10.0);
    depth_ = this->declare_parameter("depth", 10);
}
