cmake_minimum_required(VERSION 3.8)
project(turtlebro)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

add_executable(odom_node src/odom_node.cpp)
ament_target_dependencies(odom_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)

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

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RawOdom.msg"
  "srv/BoardInfo.srv"
  "srv/DomainIDService.srv"
  DEPENDENCIES std_msgs
 )

install(DIRECTORY
  launch
  urdf
  camera
  DESTINATION share/${PROJECT_NAME}/
)

# ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/odom.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  odom_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
