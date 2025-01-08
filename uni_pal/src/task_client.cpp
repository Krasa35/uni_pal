#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <TaskClient/TaskClient.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}