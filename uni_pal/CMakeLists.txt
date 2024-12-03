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
  rclpy
  uni_pal_msgs
  geometry_msgs
  ament_cmake
  ur
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

ament_python_install_package(scripts)

install(PROGRAMS
  scripts/jsonread.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
PATTERN "ARCHIVE/*" EXCLUDE 
)

ament_package()