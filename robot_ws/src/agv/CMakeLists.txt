cmake_minimum_required(VERSION 3.8)
project(agv)

find_package(ament_cmake REQUIRED)

# Install all resource directories so launch files can find meshes/urdf/config
install(DIRECTORY
  config
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
