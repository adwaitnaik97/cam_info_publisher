#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <functional>
#include <memory>

#include "helper_functions.hpp"

class CalibrationPublisher : public rclcpp::Node
{
public:
  CalibrationPublisher() : rclcpp::Node("CalibrationPublisher")
  {
  }

  // ImageTransport needs to be initialized after the node has been fully constructed
  void init()
  {
    // Create ImageTransport after node is fully constructed (this is where shared_from_this is safe to use)
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

    // Publisher
    pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

    // Params
    this->declare_parameter<int>("image_width", 2048);
    this->declare_parameter<int>("image_height", 1536);
    this->declare_parameter<std::string>("camera_name", "narrow_stereo");
    this->declare_parameter<std::string>("distortion_model", "plumb_bob");

    this->get_parameter("camera_name", camera_name_);
    this->get_parameter("distortion_model", distortion_model_);
    this->get_parameter("image_width", image_width_);
    this->get_parameter("image_height", image_height_);

    ReadMatrixFromParam("camera_matrix", camera_matrix_);
    ReadMatrixFromParam("distortion_coefficients", distortion_coefficients_);
    ReadMatrixFromParam("rectification_matrix", rectification_matrix_);
    ReadMatrixFromParam("projection_matrix", projection_matrix_);

    // Fill camera info msg
    cam_info_msg_.distortion_model = distortion_model_;
    cam_info_msg_.width = image_width_;
    cam_info_msg_.height = image_height_;
    cam_info_msg_.k = VectorToArray<9>(FlattenMatrix(camera_matrix_));
    cam_info_msg_.d = FlattenMatrixToVec(distortion_coefficients_);
    cam_info_msg_.r = VectorToArray<9>(FlattenMatrix(rectification_matrix_));
    cam_info_msg_.p = VectorToArray<12>(FlattenMatrix(projection_matrix_));

    // Subscribe to image_raw topic and bind the callback function
    image_sub_ = it_->subscribe("image_raw", 10,
                                std::bind(&CalibrationPublisher::ImageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to image_raw");
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_;
  image_transport::Subscriber image_sub_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  sensor_msgs::msg::CameraInfo cam_info_msg_;

  std::string distortion_model_;
  std::string camera_name_;
  int image_width_;
  int image_height_;

  cv::Mat camera_matrix_, distortion_coefficients_, rectification_matrix_, projection_matrix_;

  void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    cam_info_msg_.header = msg->header;
    cam_info_msg_.width = msg->width;
    cam_info_msg_.height = msg->height;
    pub_->publish(cam_info_msg_);
    RCLCPP_INFO(this->get_logger(), "Image received — publishing camera_info");
  }

  void ReadMatrixFromParam(const std::string &param, cv::Mat &matrix)
  {
    std::vector<double> data;
    int rows, cols;

    this->declare_parameter(param + ".data", std::vector<double>());
    this->declare_parameter(param + ".rows", 0);
    this->declare_parameter(param + ".cols", 0);

    this->get_parameter(param + ".data", data);
    this->get_parameter(param + ".rows", rows);
    this->get_parameter(param + ".cols", cols);

    cv::Mat(rows, cols, CV_64F, data.data()).copyTo(matrix);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  while (rclcpp::ok())
  {
    auto node = std::make_shared<CalibrationPublisher>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
  }
  return 0;
}