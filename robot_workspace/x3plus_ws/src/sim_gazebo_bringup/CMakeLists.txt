cmake_minimum_required(VERSION 3.8)
project(sim_gazebo_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Copy launch files to install directory
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Copy world files to install directory
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Copy RViz configuration files to install directory
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Run linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
