#include "rclcpp/rclcpp.hpp"
#include "simple_path_interface/msg/path_coordinates.hpp"

class PathCoordinatesSubscriber : public rclcpp::Node
{
public:
  PathCoordinatesSubscriber() : Node("path_coordinates_subscriber")
  {
    // Declare and get parameters
    max_center_points_ = this->declare_parameter<int>("max_center_points", 10);
    max_side_points_ = this->declare_parameter<int>("max_side_points", 10);

    subscription_ = this->create_subscription<simple_path_interface::msg::PathCoordinates>(
      "path_coordinates", 1, std::bind(&PathCoordinatesSubscriber::topic_callback, this, std::placeholders::_1));
    
    // Publisherの作成
    publisher_ = this->create_publisher<simple_path_interface::msg::PathCoordinates>("adjusted_path_coordinates", 10);
  }

private:
  void pad_vector(std::vector<double> &vec, size_t size) const
  {
    if (vec.size() < size) {
      vec.resize(size, vec.back());
    } else if (vec.size() > size) {
      vec.resize(size);
    }
  }

  void topic_callback(const simple_path_interface::msg::PathCoordinates::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received PathCoordinates message");

    // メッセージからコピーを作成してベクトルを調整
    auto adjusted_msg = std::make_shared<simple_path_interface::msg::PathCoordinates>(*msg);

    // Pad center line points
    pad_vector(adjusted_msg->center_x, max_center_points_);
    pad_vector(adjusted_msg->center_y, max_center_points_);

    // Pad left bound points
    pad_vector(adjusted_msg->left_bound_x, max_side_points_);
    pad_vector(adjusted_msg->left_bound_y, max_side_points_);

    // Pad right bound points
    pad_vector(adjusted_msg->right_bound_x, max_side_points_);
    pad_vector(adjusted_msg->right_bound_y, max_side_points_);

    // Log the lengths of the arrays
    RCLCPP_INFO(this->get_logger(), "Center line points: %zu", adjusted_msg->center_x.size());
    RCLCPP_INFO(this->get_logger(), "Left bound points: %zu", adjusted_msg->left_bound_x.size());
    RCLCPP_INFO(this->get_logger(), "Right bound points: %zu", adjusted_msg->right_bound_x.size());

    // 調整されたメッセージをパブリッシュ
    publisher_->publish(*adjusted_msg);
  }

  int max_center_points_;
  int max_side_points_;
  rclcpp::Subscription<simple_path_interface::msg::PathCoordinates>::SharedPtr subscription_;
  rclcpp::Publisher<simple_path_interface::msg::PathCoordinates>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathCoordinatesSubscriber>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
