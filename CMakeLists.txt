cmake_minimum_required(VERSION 3.16.3)
project(cam_info_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_FLAGS "-Wall -g ${CMAKE_CXX_FLAGS}")
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add executable
add_executable(cam_info_publisher src/cam_info_publisher.cpp)

# Link dependencies
ament_target_dependencies(cam_info_publisher
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

add_executable(test_publisher src/test_publisher.cpp)
ament_target_dependencies(test_publisher
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

include_directories(include)

# # Uncomment after including tests
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# Install the executable
install(TARGETS cam_info_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS test_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory (if it exists)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
