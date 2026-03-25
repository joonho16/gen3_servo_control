cmake_minimum_required(VERSION 3.8)
project(gen3_servo_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  gen3_servo_control/servo_joint_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME servo_joint_bridge
)

install(PROGRAMS
  gen3_servo_control/scene_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME scene_manager
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
