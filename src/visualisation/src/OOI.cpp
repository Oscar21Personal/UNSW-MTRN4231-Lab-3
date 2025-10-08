#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class MarkerPublisher : public rclcpp::Node
{
public:
    MarkerPublisher() : Node("MarkerPublisher"), count_(0)
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("markerPublisher", 10);
        timer_ = this->create_wall_timer( std::chrono::milliseconds(200), std::bind(&MarkerPublisher::timer_callback, this));
    }
private:
    void timer_callback()
    {
        auto message = visualization_msgs::msg::Marker();
        message.header.frame_id = "OOI_frame";
        message.header.stamp = this->now();
        message.ns = "basic_shapes";
        message.type = visualization_msgs::msg::Marker::SPHERE;

        message.pose.position.x = 0.0;
        message.pose.position.y = 0.0;
        message.pose.position.z = 0.0;
        message.pose.orientation.x = 0.0;
        message.pose.orientation.y = 0.0;
        message.pose.orientation.z = 0.0;
        message.pose.orientation.w = 1.0;

        message.scale.x = 0.1;
        message.scale.y = 0.1;
        message.scale.z = 0.1;

        message.color.r = 1.0f;
        message.color.g = 0.0f;
        message.color.b = 0.0f;
        message.color.a = 1.0;

        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarkerPublisher>());
    rclcpp::shutdown();
    return 0;
}