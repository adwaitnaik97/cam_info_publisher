/*******************************************************
 *
 *    _    ___  ___   ___  ___ __   __ ___  ___  ___
 *   /_\  |_ _||   \ | _ \|_ _|\ \ / /| __|| _ \/ __|
 *  / _ \  | | | |) ||   / | |  \ V / | _| |   /\__ \
 * /_/ \_\|___||___/ |_|_\|___|  \_/  |___||_|_\|___/
 *
 *
 * Copyright (C) 2025 AIOS @ AIDRIVERS Ltd - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * author = 'Adwait Naik'
 * email  = 'adwait@aidrivers.ai'
 *******************************************************/

//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

//#include <image_transport/image_transport.h>
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

  /**
   * @brief Initializes the CalibrationPublisher node.
   *
   * Sets up publishers, parameters, and subscriptions.
   * Fills the CameraInfo message with calibration data.
   */
  void init()
  {
    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

    pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

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

    cam_info_msg_.distortion_model = distortion_model_;
    cam_info_msg_.width = image_width_;
    cam_info_msg_.height = image_height_;
    cam_info_msg_.k = VectorToArray<9>(FlattenMatrix(camera_matrix_));
    cam_info_msg_.d = FlattenMatrixToVec(distortion_coefficients_);
    cam_info_msg_.r = VectorToArray<9>(FlattenMatrix(rectification_matrix_));
    cam_info_msg_.p = VectorToArray<12>(FlattenMatrix(projection_matrix_));

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

  /**
   * @brief Callback function to handle incoming images.
   *
   * Updates the CameraInfo message with the received image's header and dimensions,
   * and publishes it to the "camera_info" topic.
   *
   * @param msg Shared pointer to the incoming sensor_msgs::msg::Image message
   */
  void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    cam_info_msg_.header = msg->header;
    cam_info_msg_.width = msg->width;
    cam_info_msg_.height = msg->height;
    pub_->publish(cam_info_msg_);
    RCLCPP_INFO(this->get_logger(), "Image received — publishing camera_info");
  }

  /**
   * @brief Loads a matrix parameter from the parameter server.
   *
   * Reads matrix dimensions and values from parameters and stores them in a cv::Mat.
   *
   * @param param Name of the matrix parameter prefix
   * @param matrix Reference to a cv::Mat object where the matrix will be stored
   */
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
