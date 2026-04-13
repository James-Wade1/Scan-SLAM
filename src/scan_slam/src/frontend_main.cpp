#include "rclcpp/rclcpp.hpp"
#include "scan_slam/slam_frontend.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<scan_slam::FrontEndNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}