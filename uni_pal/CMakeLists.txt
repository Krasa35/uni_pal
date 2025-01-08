cmake_minimum_required(VERSION 3.5)
project(uni_pal)
list(APPEND CMAKE_PREFIX_PATH "/home/ws/addon_repos")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  tf2
  tf2_ros
  tf2_geometry_msgs
  rclpy
  rclcpp
  std_msgs
  uni_pal_msgs
  uni_pal_cpplib
  geometry_msgs
  ament_cmake
  ur
  ur_msgs
  moveit_task_constructor_core
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

ament_python_install_package(scripts)

install(PROGRAMS
  scripts/jsonread.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(robot_client src/robot_client.cpp)
ament_target_dependencies(robot_client uni_pal_cpplib)

add_executable(scene_client src/scene_client.cpp)
ament_target_dependencies(scene_client uni_pal_cpplib)

add_executable(robot_republisher src/robot_republisher.cpp)
ament_target_dependencies(robot_republisher uni_pal_cpplib)

add_executable(task_client src/task_client.cpp)
ament_target_dependencies(task_client uni_pal_cpplib)

install(TARGETS
  robot_client
  scene_client
  robot_republisher
  task_client
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
PATTERN "ARCHIVE/*" EXCLUDE)

ament_package()