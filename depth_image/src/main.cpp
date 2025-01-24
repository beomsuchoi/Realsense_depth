#include "depth_image/depth_image.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    // ROS2 초기화
    rclcpp::init(argc, argv);
    
    try {
        // DepthImage 노드 생성 및 실행
        auto node = std::make_shared<DepthImage>();
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