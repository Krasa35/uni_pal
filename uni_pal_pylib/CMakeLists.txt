cmake_minimum_required(VERSION 3.8)
project(uni_pal_pylib)

# Find dependencies
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(uni_pal_pylib)

ament_package()