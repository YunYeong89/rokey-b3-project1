cmake_minimum_required(VERSION 3.10)
project(m0609_rg2_bringup)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(cobot_virtual)
install(DIRECTORY launch rviz urdf xacro config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/gripper_virtual_node.py scripts/gripper_joint_state_publisher.py
  scripts/virtual_profile_loader.py DESTINATION lib/${PROJECT_NAME})
ament_package()
