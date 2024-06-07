#include "rclcpp/rclcpp.hpp"  // ROS2 C++ 클라이언트 라이브러리
#include "sensor_msgs/msg/compressed_image.hpp"  // 압축된 이미지 메시지 타입
#include "cv_bridge/cv_bridge.h"  // OpenCV와 ROS 이미지 간의 변환을 위한 cv_bridge 라이브러리
#include "opencv2/opencv.hpp"  // OpenCV 라이브러리
#include <memory>
#include <chrono>

// GStreamer 파이프라인을 사용하여 NVIDIA Jetson에서 카메라를 설정하는 소스
std::string src = "nvarguscamerasrc sensor-id=0 ! \
    video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
    format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
    width=(int)640, height=(int)360, format=(string)BGRx ! \
    videoconvert ! video/x-raw, format=(string)BGR ! appsink";

int main(int argc, char * argv[])
{
    // ROS2 초기화
    rclcpp::init(argc, argv);
    // 노드 생성
    auto node = std::make_shared<rclcpp::Node>("campub");
    // QoS 설정 (Quality of Service) - 가장 최근 10개의 메시지를 유지하고 best effort로 전달
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    // 퍼블리셔 생성: 원본, 그레이스케일, 이진 이미지 주제
    auto mypub = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile);
    auto gray_pub = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/gray", qos_profile);
    auto binary_pub = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/binary", qos_profile);
    
    // 메시지 헤더
    std_msgs::msg::Header hdr;
    // 퍼블리시할 메시지 변수
    sensor_msgs::msg::CompressedImage::SharedPtr gray_msg, binary_msg, msg;
    // 루프 주기 설정 (40Hz)
    rclcpp::WallRate loop_rate(40.0);

    // OpenCV를 사용하여 GStreamer 소스를 통해 비디오 캡처 객체 생성
    cv::VideoCapture cap(src, cv::CAP_GSTREAMER);
    // 비디오 캡처 객체를 열지 못한 경우 오류 메시지 출력 후 종료
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Could not open video!");
        rclcpp::shutdown();
        return -1;
    }
    cv::Mat frame;

    while(rclcpp::ok())  // ROS2가 실행 중인 동안 루프 실행
    {
        cap >> frame;  // 프레임 캡처
        if (frame.empty()) { 
            RCLCPP_ERROR(node->get_logger(), "frame empty"); 
            break; 
        }
        // 캡처한 프레임을 압축된 이미지 메시지로 변환하여 퍼블리시
        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
        mypub->publish(*msg);
        
        // 그레이스케일 이미지 생성 및 퍼블리시
        cv::Mat gray_frame;
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        gray_msg = cv_bridge::CvImage(hdr, "mono8", gray_frame).toCompressedImageMsg();
        gray_pub->publish(*gray_msg);
        
        // 이진 이미지 생성 및 퍼블리시
        cv::Mat binary_frame;
        cv::threshold(gray_frame, binary_frame, 128, 255, cv::THRESH_BINARY);
        binary_msg = cv_bridge::CvImage(hdr, "mono8", binary_frame).toCompressedImageMsg();
        binary_pub->publish(*binary_msg);

        loop_rate.sleep();  // 루프 주기 대기
    }
    rclcpp::shutdown();  // ROS2 종료
    return 0;
}
