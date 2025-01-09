#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <TaskClient/TaskClient.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<TaskClient>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}