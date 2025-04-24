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