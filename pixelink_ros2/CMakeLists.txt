cmake_minimum_required(VERSION 3.5)
project(pixelink_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pixelink_ros2_interfaces REQUIRED) 
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)

set(PXL_LIB_PATH $ENV{PIXELINK_SDK_LIB})
set(PXL_INC $ENV{PIXELINK_SDK_INC})

find_library(PXL_LIB PATHS ${PXL_LIB_PATH} NAMES PxLApi)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PXL_INC}
)

add_definitions(-DPIXELINK_LINUX)

add_executable(server src/pixelink_server.cpp src/pixelinkHelper.cpp include/pixelinkHelper.h include/pixelFormat.h) 
ament_target_dependencies(server
  rclcpp pixelink_ros2_interfaces)

set(EXEC_LIST server)

target_link_libraries(
	server ${PXL_LIB}
	image_transport::image_transport)


install(TARGETS
  server
  DESTINATION lib/${PROJECT_NAME})

ament_package()
