cmake_minimum_required(VERSION 3.8)
project(uni_pal_cpplib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_task_constructor_core
  ament_cmake
  rclcpp
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  uni_pal_msgs
  ur_msgs
  shape_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

add_library(${PROJECT_NAME} 
  src/RobotClient.cpp 
  src/SceneClient.cpp 
  src/TaskClient.cpp
  src/tasks/create_demo_task_.cpp
  src/tasks/create_homing_task_.cpp
  src/tasks/create_emergency_homing_task_.cpp
  src/tasks/create_pick_task_.cpp
  src/tasks/create_place_task_.cpp
  src/Controller.cpp
  src/URRepublisher.cpp 
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

install(
  DIRECTORY include/RobotClient include/SceneClient include/TaskClient include/Controller include/URRepublisher 
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()