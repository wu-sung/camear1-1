#include "rclcpp/rclcpp.hpp"  // ROS2 C++ 클라이언트 라이브러리
#include "sensor_msgs/msg/compressed_image.hpp"  // 압축된 이미지 메시지 타입
#include "opencv2/opencv.hpp"  // OpenCV 라이브러리
#include <memory>

// 콜백 함수: 구독한 메시지를 처리
void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 압축된 이미지를 디코딩하여 OpenCV Mat 객체로 변환
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    
    // 그레이스케일로 변환하여 출력
    cv::Mat gray_frame;
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
    cv::imshow("Gray Image", gray_frame);

    // 이진 이미지로 변환하여 출력
    cv::Mat binary_frame;
    cv::threshold(gray_frame, binary_frame, 128, 255, cv::THRESH_BINARY);
    cv::imshow("Binary Image", binary_frame);
    
    // 1ms 동안 키 입력 대기 (imshow 창 업데이트를 위해 필요)
    cv::waitKey(1);

    // 수신한 이미지 정보 로그 출력
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received Image : %s, %d, %d", msg->format.c_str(), frame.rows, frame.cols);
}

int main(int argc, char* argv[])
{
    // ROS2 초기화
    rclcpp::init(argc, argv);
    // 노드 생성
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl");
    // QoS 설정 (Quality of Service) - 가장 최근 10개의 메시지를 유지하고 best effort로 전달
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    // 구독자 생성: "image/compressed" 주제를 구독하며 콜백 함수 mysub_callback를 호출
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, mysub_callback);
    
    // 노드 실행 - 콜백 함수 호출을 위한 스핀
    rclcpp::spin(node);
    // ROS2 종료
    rclcpp::shutdown();
    return 0;
}
