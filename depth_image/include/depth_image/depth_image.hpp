#ifndef DEPTH_IMAGE_HPP
#define DEPTH_IMAGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <image_transport/image_transport.hpp>

class DepthImage : public rclcpp::Node
{
public:
    DepthImage();
    virtual ~DepthImage();

private:
    // RealSense 관련 멤버
    rs2::pipeline pipe;
    rs2::config cfg;
    
    // ROS2 관련 멤버
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 메서드
    void timer_callback();
    float get_distance_to_center(const rs2::depth_frame& depth_frame);
    void initialize_realsense();
};

#endif // DEPTH_IMAGE_HPP