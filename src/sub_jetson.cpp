#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>

void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data),  cv::IMREAD_COLOR);
    
    // 그레이스케일로 변환하여 출력
    cv::Mat gray_frame;
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
    cv::imshow("Gray Image", gray_frame);

    // 이진 이미지로 변환하여 출력
    cv::Mat binary_frame;
    cv::threshold(gray_frame, binary_frame, 128, 255, cv::THRESH_BINARY);
    cv::imshow("Binary Image", binary_frame);
    
    cv::waitKey(1);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received Image : %s, %d, %d", msg->format.c_str(), frame.rows, frame.cols);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_jetson");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, mysub_callback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}