cmake_minimum_required(VERSION 3.8)
project(starq_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(starq REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
  "msg/MotorState.msg"
  "msg/ODriveInfo.msg"
  "msg/LegCommand.msg"
  "msg/LegState.msg"
  DEPENDENCIES geometry_msgs
)

# include directories
include_directories(include)

# declare a C++ executable
add_executable(starq_node src/starq_node.cpp)

target_link_libraries(starq_node starq::starq)

# Specify libraries to link a library or executable target against
ament_target_dependencies(starq_node rclcpp geometry_msgs)

# Find the typesupport target for your messages
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link the typesupport target with your executable
if(cpp_typesupport_target)
  target_link_libraries(starq_node "${cpp_typesupport_target}")
endif()

# Install C++ executables
install(TARGETS
  starq_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
