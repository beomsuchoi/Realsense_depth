#include "depth_realsense/depth_realsense.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    // ROS2 초기화
    rclcpp::init(argc, argv);
    
    try {
        // DepthRealsense 노드 생성 및 실행
        auto node = std::make_shared<DepthRealsense>();
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        std::cerr << "Error occurred: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    
    // ROS2 종료
    rclcpp::shutdown();
    return 0;
}