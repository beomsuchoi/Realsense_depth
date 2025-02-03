#include "sub_realsense/sub_realsense.hpp"

ImageSubscriber::ImageSubscriber()
: Node("image_subscriber")
{
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/depth/image_rect_raw", 10,
        std::bind(&ImageSubscriber::depth_callback, this, std::placeholders::_1));
}

ImageSubscriber::~ImageSubscriber()
{
    cv::destroyAllWindows();
}

void ImageSubscriber::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        // Convert ROS message to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::Mat depth_image = cv_ptr->image;
        
        cv::Mat colored_depth = process_depth_image(depth_image);
        
        // Get distance to center point
        float center_distance = get_distance_to_center(depth_image);
        
        // Display distance information
        cv::putText(colored_depth, 
                   "Distance: " + std::to_string(center_distance) + " mm", 
                   cv::Point(20, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   1.0, 
                   cv::Scalar(255, 255, 255), 
                   2);
        
        // Show image
        cv::imshow("Depth View", colored_depth);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}

float ImageSubscriber::get_distance_to_center(const cv::Mat& depth_mat)
{
    // Get depth value at center point
    uint16_t depth_value = depth_mat.at<uint16_t>(HEIGHT/2, WIDTH/2);
    return static_cast<float>(depth_value);
}

cv::Mat ImageSubscriber::process_depth_image(const cv::Mat& depth_image)
{
    cv::Mat normalized_depth;
    cv::Mat colored_depth;
    
    // Normalize depth values for visualization (0-255)
    depth_image.convertTo(normalized_depth, CV_8UC1, 255.0 / MAX_DEPTH);
    
    // Apply color map
    cv::applyColorMap(normalized_depth, colored_depth, cv::COLORMAP_JET);
    
    // Draw center point
    cv::circle(colored_depth, 
              cv::Point(WIDTH/2, HEIGHT/2), 
              4, 
              cv::Scalar(255, 255, 255), 
              -1);
    
    return colored_depth;
}