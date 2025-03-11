//#define DEBUG_CALLBACK_FPS

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>



int main(int argc, char *argv[])
{
    cv::VideoCapture cap;
    rclcpp::init(argc, argv);

    auto camera_driver_node = rclcpp::Node::make_shared("CameraDriver");

    RCLCPP_INFO(camera_driver_node->get_logger(), "CameraDriver started");

    image_transport::ImageTransport it(camera_driver_node);
    image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);

    std::string pipeline ="nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)GRAY8 ! appsink";
    cap.open(pipeline, cv::CAP_GSTREAMER);



    if (cap.isOpened())
        while (rclcpp::ok())
        {
            cv::Mat frame;
            cap >> frame;
            if (frame.empty()) {
                RCLCPP_WARN(camera_driver_node->get_logger(), "Empty frame");
                continue;
            }

            std_msgs::msg::Header header;
            header.stamp = rclcpp::Clock().now();

            cv::cuda::GpuMat original_gpu_frame;
            original_gpu_frame.upload(frame);

            cv::cuda::GpuMat resized_gpu_frame;
            cv::cuda::resize(original_gpu_frame, resized_gpu_frame, cv::Size(1024, 576), cv::INTER_LINEAR);

            cv::Mat resized_frame;
            resized_gpu_frame.download(resized_frame);

            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", resized_frame).toImageMsg();
            msg->header = header;

            // RCLCPP_INFO(camera_driver_node->get_logger(), "Timestamp %f", header.stamp.sec + header.stamp.nanosec*1e-9);

            pub.publish(*msg);

#ifdef DEBUG_CALLBACK_FPS
            static auto last_time = std::chrono::steady_clock::now();
            static std::vector<double> fps_values;
            static constexpr size_t max_samples = 1;
            static bool first_img = true;
            auto current_time = std::chrono::steady_clock::now();
            double elapsed_time = std::chrono::duration<double>(current_time - last_time).count();

            last_time = current_time;

            if (!first_img) {

                double fps = 1.0 / elapsed_time;

                fps_values.push_back(fps);

                if (fps_values.size() > max_samples) {
                    fps_values.erase(fps_values.begin());
                }

                double avg_fps = std::accumulate(fps_values.begin(), fps_values.end(), 0.0) / fps_values.size();

                RCLCPP_INFO(rclcpp::get_logger("image_subscriber"), "Callback rate/FPS: %.2f ", avg_fps);
            }
            else
                first_img = false;
#endif
        }
    else
        RCLCPP_ERROR(camera_driver_node->get_logger(), "Failed to open camera");

    rclcpp::spin(camera_driver_node);
    rclcpp::shutdown();
    return 0;
}
