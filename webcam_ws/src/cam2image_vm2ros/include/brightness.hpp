#ifndef BRIGHTNESS_HPP
#define BRIGHTNESS_HPP

#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/core/mat.hpp"
#include "rclcpp/rclcpp.hpp"

class Brightness : public rclcpp::Node
{
    public:
        explicit Brightness(const rclcpp::NodeOptions &options);

    private:
        void initialize();
        void avg_brightness(const sensor_msgs::msg::Image::SharedPtr msg);
        void parse_parameters();

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr light_pub_;

        size_t brightness_threshold;
        bool is_light_on;
        size_t depth_;
};

#endif

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Brightness)