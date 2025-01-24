cmake_minimum_required(VERSION 3.8)
project(depth_image)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realsense2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${realsense2_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Add executable
add_executable(depth_image_node 
  src/main.cpp
  src/depth_image.cpp
)

# Find librealsense2
find_library(REALSENSE2_FOUND realsense2 REQUIRED)

# Link libraries
target_link_libraries(depth_image_node
  ${REALSENSE2_FOUND}
  ${OpenCV_LIBS}
)

# Add ROS 2 dependencies
ament_target_dependencies(depth_image_node 
  rclcpp 
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

# Install targets
install(TARGETS
depth_image_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()