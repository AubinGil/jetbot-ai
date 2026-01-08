cmake_minimum_required(VERSION 3.8)
project(robot_face_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FaceExpression.msg"
  "msg/EyeGaze.msg"
  "srv/TriggerAnimation.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install Python executables
install(PROGRAMS
  robot_face_display/robot_face_node.py
  robot_face_display/robot_face_control_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package
install(DIRECTORY robot_face_display/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
