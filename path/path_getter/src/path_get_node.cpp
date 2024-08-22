#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "simple_path_interface/msg/path_coordinates.hpp" // 修正箇所

class PathWithLaneIdSubscriber : public rclcpp::Node
{
public:
  PathWithLaneIdSubscriber() : Node("path_with_lane_id_subscriber")
  {
    subscription_ = this->create_subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>(
      "/path_with_lane_id", 1, std::bind(&PathWithLaneIdSubscriber::topic_callback, this, std::placeholders::_1));
    
    publisher_ = this->create_publisher<simple_path_interface::msg::PathCoordinates>("path_coordinates", 10);
  }

private:
  void topic_callback(const autoware_auto_planning_msgs::msg::PathWithLaneId::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received PathWithLaneId message");

    std::vector<double> center_x, center_y;
    std::vector<double> left_bound_x, left_bound_y;
    std::vector<double> right_bound_x, right_bound_y;

    // Center line points
    for (const auto & point_with_lane_id : msg->points) {
      const auto & position = point_with_lane_id.point.pose.position;
      center_x.push_back(position.x);
      center_y.push_back(position.y);
    }

    // Left bound points
    if (!msg->left_bound.empty()) {
      for (const auto & point : msg->left_bound) {
        left_bound_x.push_back(point.x);
        left_bound_y.push_back(point.y);
      }
    }

    // Right bound points
    if (!msg->right_bound.empty()) {
      for (const auto & point : msg->right_bound) {
        right_bound_x.push_back(point.x);
        right_bound_y.push_back(point.y);
      }
    }

    // Log the lengths of the arrays
    RCLCPP_INFO(this->get_logger(), "Center line points: %zu", center_x.size());
    RCLCPP_INFO(this->get_logger(), "Left bound points: %zu", left_bound_x.size());
    RCLCPP_INFO(this->get_logger(), "Right bound points: %zu", right_bound_x.size());

    // Create and publish the PathCoordinates message
    auto path_coordinates_msg = simple_path_interface::msg::PathCoordinates();
    path_coordinates_msg.center_x = center_x;
    path_coordinates_msg.center_y = center_y;
    path_coordinates_msg.left_bound_x = left_bound_x;
    path_coordinates_msg.left_bound_y = left_bound_y;
    path_coordinates_msg.right_bound_x = right_bound_x;
    path_coordinates_msg.right_bound_y = right_bound_y;

    publisher_->publish(path_coordinates_msg);
    RCLCPP_INFO(this->get_logger(), "Published PathCoordinates message");
  }

  rclcpp::Subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>::SharedPtr subscription_;
  rclcpp::Publisher<simple_path_interface::msg::PathCoordinates>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathWithLaneIdSubscriber>());
  rclcpp::shutdown();
  return 0;
}
