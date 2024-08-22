#include "rclcpp/rclcpp.hpp"
#include "ai_object_interface/msg/ai_object_pos.hpp"

class ObjectSubscriber : public rclcpp::Node
{
public:
    ObjectSubscriber()
    : Node("object_subscriber")
    {
        subscription_ = this->create_subscription<ai_object_interface::msg::AIObjectPos>(
            "/aichallenge/filtered_objects",
            10,
            std::bind(&ObjectSubscriber::topic_callback, this, std::placeholders::_1)
        );
    }

private:
    void topic_callback(const ai_object_interface::msg::AIObjectPos::SharedPtr msg)
    {
        // ここでObjectの位置情報を取得する
        RCLCPP_INFO(this->get_logger(), "Received object positions.");
        
        // 座標のサイズを8に揃えるために0埋め
        std::vector<double> x_positions = msg->x;
        std::vector<double> y_positions = msg->y;

        while (x_positions.size() < 8) {
            x_positions.push_back(0.0);
        }

        while (y_positions.size() < 8) {
            y_positions.push_back(0.0);
        }

        for (size_t i = 0; i < 8; ++i) {
            RCLCPP_INFO(this->get_logger(), "Object %zu: x=%f, y=%f", i, x_positions[i], y_positions[i]);
        }
    }

    rclcpp::Subscription<ai_object_interface::msg::AIObjectPos>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectSubscriber>());
    rclcpp::shutdown();
    return 0;
}
