#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <memory>
#include <chrono>

std::string src = "nvarguscamerasrc sensor-id=0 ! \
    video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
    format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
    width=(int)640, height=(int)360, format=(string)BGRx ! \
    videoconvert ! video/x-raw, format=(string)BGR ! appsink";

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("campub");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto mypub = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile);
    auto gray_pub = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/gray", qos_profile);
    auto binary_pub = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/binary", qos_profile);
    
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::CompressedImage::SharedPtr gray_msg, binary_msg, msg;
    rclcpp::WallRate loop_rate(40.0);

    cv::VideoCapture cap(src, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Could not open video!");
        rclcpp::shutdown();
        return -1;
    }
    cv::Mat frame;

    while(rclcpp::ok())
    {
        cap >> frame;
        if (frame.empty()) { RCLCPP_ERROR(node->get_logger(), "frame empty"); break; }
        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
        mypub->publish(*msg);
        
        // 그레이스케일 이미지 생성 및 발행
        cv::Mat gray_frame;
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        gray_msg = cv_bridge::CvImage(hdr, "mono8", gray_frame).toCompressedImageMsg();
        gray_pub->publish(*gray_msg);
        
        // 이진 이미지 생성 및 발행
        cv::Mat binary_frame;
        cv::threshold(gray_frame, binary_frame, 128, 255, cv::THRESH_BINARY);
        binary_msg = cv_bridge::CvImage(hdr, "mono8", binary_frame).toCompressedImageMsg();
        binary_pub->publish(*binary_msg);

        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}