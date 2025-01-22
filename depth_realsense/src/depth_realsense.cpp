#include "depth_realsense/depth_realsense.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

DepthRealsense::DepthRealsense() 
    : Node("depth_realsense_node")
{
    initialize_realsense();
    
    // ROS2 퍼블리셔 설정
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "depth_image", 10);
    
    // 타이머 설정 (30Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&DepthRealsense::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Depth RealSense node has been initialized");
}

DepthRealsense::~DepthRealsense() {
    pipe.stop();
    RCLCPP_INFO(this->get_logger(), "Depth RealSense node has been terminated");
}

void DepthRealsense::initialize_realsense() {
    // RealSense 설정
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    
    try {
        pipe.start(cfg);
        RCLCPP_INFO(this->get_logger(), "RealSense pipeline started successfully");
    } catch (const rs2::error & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline: %s", e.what());
        throw;
    }
}

void DepthRealsense::timer_callback() {
    try {
        // 프레임 가져오기
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        
        if (!depth_frame) {
            RCLCPP_WARN(this->get_logger(), "Failed to get depth frame");
            return;
        }
        
        // 중심점 거리 측정
        float center_distance = get_distance_to_center();
        RCLCPP_INFO(this->get_logger(), "Center distance: %.3f meters", center_distance);

        // depth 프레임을 이미지로 변환
        cv::Mat depth_image(
            cv::Size(depth_frame.get_width(), depth_frame.get_height()),
            CV_16UC1,
            (void*)depth_frame.get_data(),
            cv::Mat::AUTO_STEP
        );

        // OpenCV 이미지를 ROS 메시지로 변환
        auto depth_msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "16UC1",
            depth_image
        ).toImageMsg();

        // 메시지 헤더 설정
        depth_msg->header.stamp = this->now();
        depth_msg->header.frame_id = "camera_depth_frame";

        // 발행
        depth_pub_->publish(*depth_msg);

    } catch (const rs2::error & e) {
        RCLCPP_ERROR(this->get_logger(), "RealSense error calling: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error calling: %s", e.what());
    }
}

float DepthRealsense::get_distance_to_center() {
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::depth_frame depth = frames.get_depth_frame();
    
    if (!depth) {
        RCLCPP_WARN(this->get_logger(), "Failed to get depth frame for center distance");
        return -1.0f;
    }
    
    int width = depth.get_width();
    int height = depth.get_height();
    float distance = depth.get_distance(width/2, height/2);
    
    return distance;
}