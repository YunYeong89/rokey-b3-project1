cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PosePdConfig.msg"
  "msg/CartesianSetpoint.msg"
  "msg/CartesianState.msg"
  "srv/CartesianSession.srv"
  "msg/JointServoSetpoint.msg"
  "msg/JointServoStatus.msg"
  "srv/JointServoSession.srv"
  "msg/ContactSample.msg"
  "msg/ContactSampleArray.msg"
  "msg/RoiRegion.msg"
  "msg/RoiArray.msg"
  "action/ScanPart.action"
  "msg/LiftStatus.msg"
  "srv/LiftCommand.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
