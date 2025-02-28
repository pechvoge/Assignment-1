#ifndef SEQUENCE_GENERATOR_HPP
#define SEQUENCE_GENERATOR_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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
        void publisherCallback();       
        
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
        rclcpp::TimerBase::SharedPtr pub_timer_;

        rclcpp::Time init_time;
        size_t depth_;
        float twist_strength_;
        geometry_msgs::msg::Twist random_twist;
};

#endif // SEQUENCE_GENERATOR_HPP
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Sequence_generator_node)