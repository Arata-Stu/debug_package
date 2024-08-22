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

    // Pad center line points
    auto center_x = msg->center_x;
    auto center_y = msg->center_y;
    pad_vector(center_x, max_center_points_);
    pad_vector(center_y, max_center_points_);

    // Pad left bound points
    auto left_bound_x = msg->left_bound_x;
    auto left_bound_y = msg->left_bound_y;
    pad_vector(left_bound_x, max_side_points_);
    pad_vector(left_bound_y, max_side_points_);

    // Pad right bound points
    auto right_bound_x = msg->right_bound_x;
    auto right_bound_y = msg->right_bound_y;
    pad_vector(right_bound_x, max_side_points_);
    pad_vector(right_bound_y, max_side_points_);

    // Log the lengths of the arrays
    RCLCPP_INFO(this->get_logger(), "Center line points: %zu", center_x.size());
    RCLCPP_INFO(this->get_logger(), "Left bound points: %zu", left_bound_x.size());
    RCLCPP_INFO(this->get_logger(), "Right bound points: %zu", right_bound_x.size());

    // Here you can process the padded vectors as needed
  }

  int max_center_points_;
  int max_side_points_;
  rclcpp::Subscription<simple_path_interface::msg::PathCoordinates>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathCoordinatesSubscriber>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
