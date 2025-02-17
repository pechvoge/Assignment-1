#ifndef BRIGHTNESS_HPP
#define BRIGHTNESS_HPP

#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "rclcpp/rclcpp.hpp"

class Brightness : public rclcpp::Node
{
    public:
        explicit Brightness(const rclcpp::NodeOptions &options);

    private:
        void initialize();

        rclcpp::Subscription<std_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr light_pub_;

        float brightness;
        size_t brightness_threshold;
        bool is_light_on;
        size_t depth_;
        std::string input_topic_;
        std::string output_topic_;
}

#endif