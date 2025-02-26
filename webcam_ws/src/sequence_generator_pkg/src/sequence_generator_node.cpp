#include "sequence_generator_pkg/sequence_generator_node.hpp"

Sequence_generator_node::Sequence_generator_node(const rclcpp::NodeOptions &options)
    : Node("sequence_generator_node", options)
{
    parse_parameters();
    initialize();
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
    camera_location_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/output/camera_position", qos, std::bind(&Sequence_generator_node::updateCameraCoords, this, std::placeholders::_1));
    robo_location_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/output/robot_pose", qos, std::bind(&Sequence_generator_node::updateRoboCoords, this, std::placeholders::_1));
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/input/twist", qos);

    desired_point.x = 90;
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
    
    compute_timer_ = this->create_wall_timer(
       std::chrono::milliseconds(static_cast<int>(1000.0 / compute_freq_)),
       std::bind(&Sequence_generator_node::Sequence_generator, this));
    pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / pub_freq_)),
        std::bind(&Sequence_generator_node::publisherCallback, this));

}

void Sequence_generator_node::updateRoboCoords(const geometry_msgs::msg::PoseStamped::SharedPtr roboPosMsg)
{
    x0_ = roboPosMsg->pose.position.x;
    theta_z0_ = roboPosMsg->pose.orientation.z;
    //RCLCPP_INFO(get_logger(), "Robo x: %f, Robo theta_z: %f", x0_, theta_z0_);
}

void Sequence_generator_node::updateCameraCoords(const geometry_msgs::msg::PointStamped::SharedPtr camPosMsg)
{
    pix_x0_ = camPosMsg->point.x;
    pix_y0_ = camPosMsg->point.y;
    //RCLCPP_INFO(get_logger(), "Camera x: %f, Camera y: %f", pix_x0_, pix_y0_);
}

void Sequence_generator_node::desiredPose()
{
    float delta_pix_x = pix_x0_ - desired_point_pointer->x;
    float delta_pix_y = pix_y0_ - desired_point_pointer->y;
    RCLCPP_INFO(get_logger(), "Des x: %f, Des y: %f", desired_point_pointer->x, desired_point_pointer->y);
    absolute(delta_pix_y);
    // define as conversion factor between window size
    theta_zf_ = 2.0 / 230.0 * delta_pix_x; // width of window / 2 - width of sub image / 2= 320 - 90 = 230 and max angle is 2
    xf_ = -5.0 / 180.0 * delta_pix_y;      // delta y at the top of the window = 180 and zoom is -5
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
    //RCLCPP_INFO(get_logger(), "Seq Camera x: %f, Seq Camera y: %f", pix_x0_, pix_y0_);
    desiredPose();
    std::array<float, 4> a = getCoefficients(final_time, theta_z0_, theta_zf_);
    std::array<float, 4> b = getCoefficients(final_time, x0_, xf_);
    //RCLCPP_INFO(get_logger(), "Coefficients a: %f, %f, %f, %f", a[0], a[1], a[2], a[3]);
    //RCLCPP_INFO(get_logger(), "Coefficients b: %f, %f, %f, %f", b[0], b[1], b[2], b[3]);
    twist_buffer.clear();
    float t = dt * buffer_size;
    geometry_msgs::msg::Twist twist;
    for (size_t i = 0; i < buffer_size; i++)
    {
        t -= dt;
        twist.angular.z = a[1] + 2 * a[2] * t + 3 * a[3] * power(t, 2);
        twist.linear.x = b[1] + 2 * b[2] * t + 3 * b[3] * power(t, 2);
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist_buffer.push_back(twist);
        //RCLCPP_INFO(get_logger(), "Time: %f, Twist angular.z: %f, Twist linear.x: %f", t, twist.angular.z, twist.linear.x);
    }
}

void Sequence_generator_node::publisherCallback()
{
    if (twist_buffer.empty())
    {
        twist_pub_->publish(zero_twist);
        zeroTwistCounter++;
       // RCLCPP_INFO(get_logger(), "Zero twist published %d times", zeroTwistCounter);
    }
    else
    {
        twist_pub_->publish(twist_buffer.back());
        geometry_msgs::msg::Twist testTwist;
        testTwist = twist_buffer.back();
        //RCLCPP_INFO(get_logger(), "TestTwist angular.z: %f, TestTwist linear.x: %f", testTwist.angular.z, testTwist.linear.x);
        twist_buffer.pop_back();
        bufferTwistCounter++;
        //RCLCPP_INFO(get_logger(), "Buffer twist published %d times", bufferTwistCounter);
    }
}

void Sequence_generator_node::parse_parameters()
{
    //pub_freq_ = this->declare_parameter("publisher_frequency", 10.0);
    depth_ = this->declare_parameter("depth", 10);
}
