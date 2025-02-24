#ifndef OBJECT_DETECTION_NODE_HPP
#define OBJECT_DETECTION_NODE_HPP

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/core/mat.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "opencv2/highgui.hpp"


class Object_detection_node : public rclcpp::Node
{
    public:
        explicit Object_detection_node(const rclcpp::NodeOptions &options);

    private:
        void initialize();
        void parse_parameters();
        void CoG_determiner(const sensor_msgs::msg::Image::SharedPtr msg);
        
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr CoG_pub_;

        size_t gray_threshold_;
        size_t depth_;
};

#endif // OBJECT_DETECTION_NODE_HPP

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Object_detection_node)