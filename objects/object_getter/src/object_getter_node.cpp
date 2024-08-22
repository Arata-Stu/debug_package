#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ai_object_interface/msg/ai_object_pos.hpp"

class ObjectSubscriberPublisher : public rclcpp::Node
{
public:
    ObjectSubscriberPublisher()
    : Node("object_subscriber_publisher")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "objects",
            1,
            std::bind(&ObjectSubscriberPublisher::topic_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<ai_object_interface::msg::AIObjectPos>("filtered_objects", 10);
    }

private:
    void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        size_t data_size = msg->data.size();
        if (data_size < 2) {
            RCLCPP_WARN(this->get_logger(), "Received data with insufficient elements.");
            return;
        }

        auto xy_array_msg = ai_object_interface::msg::AIObjectPos();
        xy_array_msg.header.stamp = this->get_clock()->now();
        xy_array_msg.header.frame_id = "map";

        for (size_t i = 0; i < data_size / 4; ++i) {
            size_t base_index = i * 4;
            if (base_index + 1 < data_size) {
                xy_array_msg.x.push_back(msg->data[base_index]);
                xy_array_msg.y.push_back(msg->data[base_index + 1]);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Publishing %zu xy pairs", xy_array_msg.x.size());
        publisher_->publish(xy_array_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<ai_object_interface::msg::AIObjectPos>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectSubscriberPublisher>());
    rclcpp::shutdown();
    return 0;
}
