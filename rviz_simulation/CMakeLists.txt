cmake_minimum_required(VERSION 3.8)
project(rviz_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files, URDF, config, and mesh files
install(
  DIRECTORY
    launch
    urdf
    config
    meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()