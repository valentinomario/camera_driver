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

    std::string output_topic, format;
    int input_height, input_width, input_framerate;
    int output_height, output_width;
    int sensor_id;

    cv::VideoCapture cap;
    rclcpp::init(argc, argv);

    auto camera_driver_node = rclcpp::Node::make_shared("camera_driver");

    camera_driver_node->declare_parameter("outputTopic", "/camera/image_raw");
    camera_driver_node->get_parameter("outputTopic", output_topic);

    camera_driver_node->declare_parameter("inputHeight", 1080);
    camera_driver_node->get_parameter("inputHeight", input_height);

    camera_driver_node->declare_parameter("inputWidth", 1920);
    camera_driver_node->get_parameter("inputWidth", input_width);

    camera_driver_node->declare_parameter("inputFramerate", 30);
    camera_driver_node->get_parameter("inputFramerate", input_framerate);

    camera_driver_node->declare_parameter("outputHeight", 576);
    camera_driver_node->get_parameter("outputHeight", output_height);

    camera_driver_node->declare_parameter("outputWidth", 1024);
    camera_driver_node->get_parameter("outputWidth", output_width);

    camera_driver_node->declare_parameter("sensorId", 0);
    camera_driver_node->get_parameter("sensorId", sensor_id);

    camera_driver_node->declare_parameter("format", "GRAY8");
    camera_driver_node->get_parameter("format", format);


    RCLCPP_INFO(camera_driver_node->get_logger(), "camera driver started");

    image_transport::ImageTransport it(camera_driver_node);
    image_transport::Publisher pub = it.advertise(output_topic, 1);

    std::string pipeline ="nvarguscamerasrc sensor_id=" + std::to_string(sensor_id) +
            " ! video/x-raw(memory:NVMM),width=" + std::to_string(input_width)+ ",height=" + std::to_string(input_height) + ",framerate=" + std::to_string(input_framerate) +
            "/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)" + format + " ! appsink";

    RCLCPP_INFO(camera_driver_node->get_logger(), "%s", pipeline.c_str());

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
            cv::cuda::resize(original_gpu_frame, resized_gpu_frame, cv::Size(output_width, output_height), cv::INTER_LINEAR);

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
