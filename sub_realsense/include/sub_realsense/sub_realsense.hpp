#ifndef SUB_REALSENSE_HPP_
#define SUB_REALSENSE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber();
    ~ImageSubscriber();

private:
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    float get_distance_to_center(const cv::Mat& depth_mat);
    cv::Mat process_depth_image(const cv::Mat& depth_image);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    
    const int WIDTH = 640;
    const int HEIGHT = 480;
    const float MAX_DEPTH = 10000.0;
};

#endif  // SUB_REALSENSE_HPP_