cmake_minimum_required(VERSION 3.16.3)
project(cam_info_publisher)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -g ${CMAKE_CXX_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define the path to the dynamically generated test_publisher.cpp
set(TEST_PUBLISHER_SRC "${CMAKE_CURRENT_BINARY_DIR}/test_publisher.cpp")

# Custom command to generate test_publisher.cpp with the necessary content
add_custom_command(
  OUTPUT ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "#include <cv_bridge/cv_bridge.h>" > ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "#include \"image_transport/image_transport.hpp\"" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "#include \"opencv2/highgui/highgui.hpp\"" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "#include \"rclcpp/rclcpp.hpp\"" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "int main(int argc, char ** argv) {" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "  rclcpp::init(argc, argv);" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(\"image_publisher\");" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "  image_transport::ImageTransport it(node);" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "  image_transport::Publisher pub = it.advertise(\"image_raw\", 1);" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "  cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "  std_msgs::msg::Header hdr;" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "  sensor_msgs::msg::Image::SharedPtr msg;" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "  msg = cv_bridge::CvImage(hdr, \"bgr8\", image).toImageMsg();" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "  rclcpp::WallRate loop_rate(5);" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "  while (rclcpp::ok()) {" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "    pub.publish(msg);" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "    rclcpp::spin_some(node);" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "    loop_rate.sleep();" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "  }" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "  return 0;" >> ${TEST_PUBLISHER_SRC}
  COMMAND ${CMAKE_COMMAND} -E echo "}" >> ${TEST_PUBLISHER_SRC}
  COMMENT "Generating test_publisher.cpp with content"
  VERBATIM
)

# Create a custom target to ensure the file generation happens first
add_custom_target(generate_test_publisher ALL DEPENDS ${TEST_PUBLISHER_SRC})

# Add executable for cam_info_publisher
add_executable(cam_info_publisher src/cam_info_publisher.cpp)

# Link dependencies for cam_info_publisher
ament_target_dependencies(cam_info_publisher
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

# Add executable for test_publisher (it depends on the dynamically generated file)
add_executable(test_publisher ${TEST_PUBLISHER_SRC})

# Link dependencies for test_publisher
ament_target_dependencies(test_publisher
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

# Ensure that test_publisher depends on the custom target
add_dependencies(test_publisher generate_test_publisher)

# Include directories
include_directories(include)

# Install the executables
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
