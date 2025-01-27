#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <RobotClient/RobotClient.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotClient>());
  rclcpp::shutdown();
  return 0;
}