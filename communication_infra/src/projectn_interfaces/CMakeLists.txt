cmake_minimum_required(VERSION 3.8)
project(projectn_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---- Messages and Services we generate ----
set(MSG_FILES
  msg/DataMsg.msg
  msg/RMsg.msg
)

set(SRV_FILES
  srv/RegisterChild.srv
  srv/GetMsgId.srv
  srv/SubmitDm.srv
)

# ---- Generate code ----
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

