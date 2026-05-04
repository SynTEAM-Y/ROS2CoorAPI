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

# Copy map files (used by robot_rviz.launch.py via the `map` argument)
install(
  DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# Copy RViz configuration files to install directory
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# ===== Isolated overrides (your work, kept inside this package) =====
# These install our modified URDF, meshes, and Python nodes from scripts/
# WITHOUT touching the upstream packages (yahboomcar_description / x3plus_examples).

# Install meshes used by our URDF
install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Rewrite mesh URIs in the URDF xacro from
#   package://yahboomcar_description/meshes/...
# to
#   package://sim_gazebo_bringup/meshes/...
# at configure time so the original file in scripts/ is left untouched.
file(READ
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro"
  _XACRO_CONTENT
)
string(REPLACE
  "package://yahboomcar_description/meshes/"
  "package://sim_gazebo_bringup/meshes/"
  _XACRO_CONTENT "${_XACRO_CONTENT}"
)
file(WRITE
  "${CMAKE_CURRENT_BINARY_DIR}/generated/urdf/yahboomcar_X3plus.urdf.xacro"
  "${_XACRO_CONTENT}"
)
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/generated/urdf/yahboomcar_X3plus.urdf.xacro"
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install our Python nodes as ROS executables. They become runnable as:
#   ros2 run sim_gazebo_bringup manual_control
#   ros2 run sim_gazebo_bringup arm_controller
#   ros2 run sim_gazebo_bringup gripper_mimic_relay
# Each is installed individually so we can RENAME the .py off.
install(PROGRAMS scripts/x3plus_examples/manual_control.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME manual_control)
install(PROGRAMS scripts/x3plus_examples/arm_controller.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME arm_controller)
install(PROGRAMS scripts/x3plus_examples/gripper_mimic_relay.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME gripper_mimic_relay)

# Run linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
