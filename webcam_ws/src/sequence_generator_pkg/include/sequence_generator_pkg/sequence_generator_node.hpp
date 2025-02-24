#ifndef SEQUENCE_GENERATOR_HPP
#define SEQUENCE_GENERATOR_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <chrono>
#include <array>
#include <vector>
#include "rclcpp/rclcpp.hpp"


class Sequence_generator_node : public rclcpp::Node
{
    public:
        explicit Sequence_generator_node(const rclcpp::NodeOptions &options);

    private:
        void initialize();
        void parse_parameters();
        void absolute(float &delta_x);
        float power(float base, float exponent);
        std::array<float,2> desiredPose();
        std::array<float,4> getCoefficients(float final_time, float ini_q, float final_q);
        void Sequence_generator();
        void publisherCallback();       

        // 1.2.2
        //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
        rclcpp::TimerBase::SharedPtr pub_timer_;

        geometry_msgs::msg::Point desired_point;
        const geometry_msgs::msg::Point::SharedPtr desired_point_pointer = std::make_shared<geometry_msgs::msg::Point>(desired_point); 
        size_t depth_;
        float theta_z0_ = 0.0;
        float x0_ = 0.0;
        float theta_zf_;
        float xf_;
        float pix_x0_;
        float pix_y0_;
        float pub_freq_ = 30.0;
        float final_time = 6.0; // seconds
        float dt = 0.3; // seconds, equal to time constant of 1st order system
        size_t buffer_size = 10; // buffer size for twist buffer to be published
        geometry_msgs::msg::Twist zero_twist;
        std::vector<geometry_msgs::msg::Twist> twist_buffer;

};

#endif // SEQUENCE_GENERATOR_HPP
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Sequence_generator_node)