cmake_minimum_required(VERSION 3.8)
project(camera_driver)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED)

add_executable(camera_driver_exec src/camera_driver.cpp)
ament_target_dependencies(camera_driver_exec rclcpp rcpputils std_msgs sensor_msgs image_transport cv_bridge OpenCV)

install(TARGETS camera_driver_exec
  DESTINATION lib/${PROJECT_NAME})

ament_package()
