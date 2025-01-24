#include "depth_image/depth_image.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

DepthImage::DepthImage()
    : Node("depth_image_node")
{
    initialize_realsense();

    // ROS2 퍼블리셔 설정
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "depth_image", 10);

    // 타이머 설정 (30Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&DepthImage::timer_callback, this));

    // create_wall_timer 함수에 timer와 실행시킬 함수를 전달하면 주기적 실행을 할 수 있음
    // create_wall_timer는 3개의 매개변수를 std::bind를 이용해서 넘겨야 함
    // 0.033초마다 timer_callback 함수를 실행하도록 설정

    RCLCPP_INFO(this->get_logger(), "Depth Image node has been initialized");
}

DepthImage::~DepthImage()
{
    pipe.stop();
    RCLCPP_INFO(this->get_logger(), "Depth Image node has been terminated");
}

/*
void DepthImage::initialize_realsense()
{
    // RealSense 설정
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    try
    {
        pipe.start(cfg);
        RCLCPP_INFO(this->get_logger(), "RealSense pipeline started successfully");
    }
    catch (const rs2::error &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline: %s", e.what());
        throw;
    }
}
*/

void DepthImage::initialize_realsense() {
    rs2::context ctx;
    auto devices = ctx.query_devices();
    if (devices.size() == 0) {
        RCLCPP_ERROR(this->get_logger(), "No RealSense devices found!");
        throw std::runtime_error("No RealSense devices found!");
    }

    // Stream 설정
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_device(devices[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    
    try {
        pipe.start(cfg);
        // 안정화를 위해 처음 몇 프레임 버리기
        for(int i = 0; i < 30; i++) {
            pipe.wait_for_frames();
        }
        RCLCPP_INFO(this->get_logger(), "RealSense pipeline started successfully");
    } catch (const rs2::error & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline: %s", e.what());
        throw;
    }
}

void DepthImage::timer_callback()
{
    try
    {
        rs2::frameset frames = pipe.wait_for_frames(1000); // 타임아웃 증가
        auto depth_frame = frames.get_depth_frame();

        if (!depth_frame) return;

        // 깊이 값 처리
        cv::Mat depth_image(
            cv::Size(depth_frame.get_width(), depth_frame.get_height()),
            CV_16UC1,
            (void *)depth_frame.get_data(),
            cv::Mat::AUTO_STEP);

        // 범위 필터링 적용
        cv::Mat filtered;
        cv::inRange(depth_image, 1, 10000, filtered); // 1mm ~ 10m 범위만 표시
        depth_image.setTo(0, ~filtered);

        // 정규화 및 컬러맵
        cv::Mat depth_normalized;
        depth_image.convertTo(depth_normalized, CV_8UC1, 255.0 / 10000.0); // 10m를 최대값으로
        
        cv::Mat depth_colormap;
        cv::applyColorMap(depth_normalized, depth_colormap, cv::COLORMAP_JET);
        
        // 유효하지 않은 깊이값 처리
        cv::Mat mask = depth_normalized == 0;
        depth_colormap.setTo(cv::Scalar(0, 0, 0), mask);

        // 중심점 표시
        int centerX = depth_frame.get_width() / 2;
        int centerY = depth_frame.get_height() / 2;
        cv::circle(depth_colormap, cv::Point(centerX, centerY), 4, cv::Scalar(0, 0, 0), -1);

        cv::imshow("Depth Image", depth_colormap);
        cv::waitKey(1);

        // ROS 메시지 발행
        auto depth_msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "16UC1", depth_image).toImageMsg();
        depth_msg->header.stamp = this->now();
        depth_msg->header.frame_id = "camera_depth_frame";
        depth_pub_->publish(*depth_msg);

        float center_distance = get_distance_to_center(depth_frame);
        RCLCPP_INFO(this->get_logger(), "Center distance: %.3f meters", center_distance);
    }
    catch (const rs2::error &e)
    {
        RCLCPP_ERROR(this->get_logger(), "RealSense error: %s", e.what());
    }
}

float DepthImage::get_distance_to_center(const rs2::depth_frame& depth_frame)
{
    if (!depth_frame) return -1.0f;

    int width = depth_frame.get_width();
    int height = depth_frame.get_height();
    float distance = depth_frame.get_distance(width / 2, height / 2);

    return distance;
}