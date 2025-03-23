# Camera Driver for ROS 2

This ROS 2 Humble package implements a GPU-accelerated camera driver for NVIDIA Jetson platforms. It captures video from a camera sensor using GStreamer, processes frames using OpenCV with CUDA acceleration, and publishes images over ROS 2.

The driver was specifically meant to run for the [LiDAR-Visual-Inertial-SLAM Project](https://github.com/valentinomario/LiDAR-Visual-Inertial-SLAM).

## Dependencies

- ROS 2 Humble
- OpenCV with CUDA support
- GStreamer

## Installation

Clone this repository into your ROS 2 workspace:
```bash
git clone <repository-url> ~/ros2_ws/src
colcon build --symlink-install
```

## Usage

Launch the camera driver with:
```bash
ros2 launch camera_driver camera_driver.launch.py
```

## Configuration

Adjust parameters in `driver_options.yaml` located in the `config` directory.

## Published Topics

- `/camera/image_raw` (`sensor_msgs/msg/Image`): Raw grayscale image stream from the camera.

