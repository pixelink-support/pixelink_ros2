cmake_minimum_required(VERSION 3.8)
project(pixelink_ros2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NoParams.srv"
  "srv/SetParams.srv"
  "srv/SetParamsRoi.srv"
  "srv/GetParams.srv"
  "srv/GetParamsRoi.srv"
  "srv/GetSerialNumbers.srv"
  "srv/SerialParam.srv"
  "srv/GetCameraInfo.srv"
  "srv/GetParamsPixelAddressing.srv"
  "srv/SetParamsPixelAddressing.srv"
  "srv/CaptureParams.srv"
  "srv/CaptureVideoParams.srv"
  "srv/GetSupportedPixelFormats.srv"
  "srv/GetPixelFormat.srv"
  "srv/SetPixelFormat.srv"
  "srv/SetTriggerParams.srv"
  "srv/SetGpioParams.srv"
  "srv/GetRange.srv"
  "srv/GetRoiRange.srv"
 )
ament_export_dependencies(rosidl_default_runtime)

ament_package()
