#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdometrySubscriber : public rclcpp::Node
{
public:
  OdometrySubscriber()
  : Node("odometry_subscriber")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", 1, std::bind(&OdometrySubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
    // Time
    rclcpp::Time time = msg->header.stamp;
    RCLCPP_INFO(this->get_logger(), "Time: %u.%u", time.seconds(), time.nanoseconds());

    // Position
    auto position = msg->pose.pose.position;
    RCLCPP_INFO(this->get_logger(), "Position: x=%f, y=%f, z=%f", position.x, position.y, position.z);

    // Orientation
    auto orientation = msg->pose.pose.orientation;
    RCLCPP_INFO(this->get_logger(), "Orientation: x=%f, y=%f, z=%f, w=%f", orientation.x, orientation.y, orientation.z, orientation.w);

    // Twist - Linear
    auto linear = msg->twist.twist.linear;
    RCLCPP_INFO(this->get_logger(), "Twist (Linear): x=%f, y=%f, z=%f", linear.x, linear.y, linear.z);

    // Twist - Angular
    auto angular = msg->twist.twist.angular;
    RCLCPP_INFO(this->get_logger(), "Twist (Angular): x=%f, y=%f, z=%f", angular.x, angular.y, angular.z);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometrySubscriber>());
  rclcpp::shutdown();
  return 0;
}
