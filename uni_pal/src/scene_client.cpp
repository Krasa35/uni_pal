#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <SceneClient/SceneClient.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SceneClient>());
  rclcpp::shutdown();
  return 0;
}