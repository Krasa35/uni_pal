cmake_minimum_required(VERSION 3.5)
project(uni_pal_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes rviz config DESTINATION share/${PROJECT_NAME}
)
install(
  FILES .setup_assistant  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()