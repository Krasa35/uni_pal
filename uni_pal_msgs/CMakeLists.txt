cmake_minimum_required(VERSION 3.8)
project(uni_pal_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs  REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStaticInfo.msg"
  "msg/RobotDynamicInfo.msg"
  "msg/RobotState.msg"
  "msg/RobotDigital.msg"
  "msg/RobotAnalog.msg"
  "msg/RobotData.msg"
  "msg/RobotPose.msg"
  "msg/RobotPredefined.msg"
  "msg/RobotRPY.msg"
  "msg/FramePose.msg"
  "srv/GetPalParams.srv"
  "srv/GetPlacePos.srv"
  "srv/SetFrame.srv"
  DEPENDENCIES geometry_msgs sensor_msgs 
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
