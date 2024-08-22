#include "rclcpp/rclcpp.hpp"
#include "ai_object_interface/msg/ai_object_pos.hpp"
#include <fstream>

class ObjectSubscriberPublisher : public rclcpp::Node
{
public:
    ObjectSubscriberPublisher()
    : Node("object_subscriber_publisher"), initial_data_received_(false)
    {
        subscription_ = this->create_subscription<ai_object_interface::msg::AIObjectPos>(
            "/aichallenge/filtered_objects",
            1,
            std::bind(&ObjectSubscriberPublisher::topic_callback, this, std::placeholders::_1)
        );

        csv_file_.open("./../coordinates.csv", std::ios::out | std::ios::app);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file.");
        } else {
            RCLCPP_INFO(this->get_logger(), "CSV file opened successfully.");
        }
    }

    ~ObjectSubscriberPublisher() {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    void topic_callback(const ai_object_interface::msg::AIObjectPos::SharedPtr msg)
    {
        if (msg->x.size() < 8 || msg->y.size() < 8) {
            RCLCPP_INFO(this->get_logger(), "Waiting for 8 coordinates...");
            return;
        }

        if (!initial_data_received_) {
            RCLCPP_INFO(this->get_logger(), "Received initial data with 8 coordinates.");
            write_to_csv("Initial", msg->x, msg->y);
            previous_x_ = msg->x;
            previous_y_ = msg->y;
            initial_data_received_ = true;
            return;
        }

        for (size_t i = 0; i < msg->x.size(); ++i) {
            if (previous_x_[i] != msg->x[i] || previous_y_[i] != msg->y[i]) {
                RCLCPP_INFO(this->get_logger(), "Coordinate change detected at index %zu: (%f, %f) -> (%f, %f)",
                            i, previous_x_[i], previous_y_[i], msg->x[i], msg->y[i]);
                write_to_csv(std::to_string(i), {msg->x[i]}, {msg->y[i]});
                previous_x_[i] = msg->x[i];
                previous_y_[i] = msg->y[i];
            }
        }
    }

    void write_to_csv(const std::string& event, const std::vector<double>& x, const std::vector<double>& y) {
        if (csv_file_.is_open()) {
            for (size_t i = 0; i < x.size(); ++i) {
                csv_file_ << event << "," << i << "," << x[i] << "," << y[i] << "\n";
            }
            csv_file_.flush();
        } else {
            RCLCPP_ERROR(this->get_logger(), "CSV file is not open.");
        }
    }

    rclcpp::Subscription<ai_object_interface::msg::AIObjectPos>::SharedPtr subscription_;
    std::vector<double> previous_x_;
    std::vector<double> previous_y_;
    bool initial_data_received_;
    std::ofstream csv_file_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectSubscriberPublisher>());
    rclcpp::shutdown();
    return 0;
}
