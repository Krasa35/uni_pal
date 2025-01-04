#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <URRepublisher/URRepublisher.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<URRepublisher>());
  rclcpp::shutdown();
  return 0;
}