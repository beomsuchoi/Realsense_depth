#ifndef DEPTH_REALSENSE_HPP
#define DEPTH_REALSENSE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <image_transport/image_transport.hpp>

class DepthRealsense : public rclcpp::Node
{
public:
    DepthRealsense();
    virtual ~DepthRealsense();

private:
    // RealSense 관련 멤버
    rs2::pipeline pipe;
    rs2::config cfg;
    
    // ROS2 관련 멤버
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 메서드
    void timer_callback();
    float get_distance_to_center();
    void initialize_realsense();
};

#endif // DEPTH_REALSENSE_HPP