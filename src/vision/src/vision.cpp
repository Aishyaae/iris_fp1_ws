#include <memory>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace cv;
using namespace std;

class VisionNode : public rclcpp::Node {
public:
  int h[2] = {19, 92}, s[2] = {25, 98}, v[2] = {118, 246};

  VisionNode() : Node("vision_node") {

    // trackbar
    // namedWindow("Control");

    // createTrackbar("Hmin", "Control", &h[0], 255);
    // createTrackbar("Smin", "Control", &s[0], 255);
    // createTrackbar("Vmin", "Control", &v[0], 255);

    // createTrackbar("Hmax", "Control", &h[1], 255);
    // createTrackbar("Smax", "Control", &s[1], 255);
    // createTrackbar("Vmax", "Control", &v[1], 255);


    // Quality of Service (QoS) disesuaikan agar cocok dengan publisher gazebo camera
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    // Subscribe ke topic kamera depan
    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
      "/catvehicle/camera_front/image_raw_front",
      qos_profile,
      std::bind(&VisionNode::image_callback, this, std::placeholders::_1)
    );
  }

private:

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      
      // Konversi dari ROS Image Message ke OpenCV Matrix (BGR8)
      cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

      cv::Mat frame_bev, frame_hsv, frame_threshold, frame_output;
      
      // bev
      int width = frame.cols;
      int height = frame.rows;

      Point2f src[4] = {
        Point2f(316, 460),
        Point2f(646, 472),
        Point2f(793, 616),
        Point2f(11, 537),
      };

      Point2f dst[4] = {
        Point2f(0.0f, 0.0f), // Kiri atas
        Point2f(static_cast<float>(width), 0.0f), // Kanan atas
        Point2f(static_cast<float>(width), static_cast<float>(height)), // kanan bawah
        Point2f(0.0f, static_cast<float>(height)) // Kiri bawah
      };

      Mat BEV_plot = getPerspectiveTransform(src, dst);
      warpPerspective(frame, frame_bev, BEV_plot, Size(width, height));

      cv::cvtColor(frame_bev, frame_hsv, cv::COLOR_BGR2HSV);

      // thereshold
      inRange(frame_hsv, cv::Scalar(h[0], s[0], v[0]), cv::Scalar(h[1], s[1], v[1]), frame_threshold);

      // contour
      vector<vector<Point>> contours;
      vector<Vec4i>> hierarcy;
      findContours(frame_threshold, contours, hierarcy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

      // titik acuan tengah bawah
      Point bottom_center(frame_threshold.cols / 2, frame_threshold.rows);
      circle(frame_output, bottom_center, 6, Scalar(255, 0, 0), -1);

      for (size_t i = 0; i < contours.size(); i++) {
        // filter luas minimal contour
        double area = contourArea(contours[i]);
        if (area < 100) continue;

        // centroid
        Moment m = moments(contours[i]);
        if (m.m00 == 0) continue;
        Point centroid(static_cast<int>(m.m10 / m.m00), static_cast<int>(m.m01 / m.m00));

        // sudut ke tengah bawah gambar
        // vector dari bottom_center ke centoroid
        double dx = centroid.x - bottom_center.x;
        double dy = bottom_center.y - centroid.y;

        // hitung sudut dalam derajat terhadap garis vertikal lurus ke atas (0 derajat = tegak lurus)
        double angle_rad = atan2(dx, dy); // 0 derajat jika sejajar vertikal, + kanan, - kiri
        double angle_deg = angle_rad * (180 / M_PI);

        // filter berdasarkan sudut toleransi
        if (abs(angle_deg) < 20) {
          drawContours(frame_output, contours, static_cast<int>(i), cv::Scalar(0, 0, 255), 2); // gambar contour
          circle(frame_output, centroid, 5, Scalar(0, 255, 0), -1); // titik tengah contour
          line(frame_output, bottom_center, centroid, Scalar(0, 255, 255), 2); // gambar garis dari tengah bawah ke centroid

          string text = format("%.1f deg", angle_deg);
          putText(frame_output, text, Point(centroid.x + 10, centroid.y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
        } else {
          drawContours(frame_output, contours, static_cast<int>(i), cv::Scalar(100, 100, 100), 1);
        }
      }
      // if(!contours.empty()) {
      //   printf("Ada contour");
      // }

      cv::imshow("RAW", frame);
      cv::imshow("Thresholding", frame_threshold);
      cv::imshow("BEV", frame_bev);
      
      cv::waitKey(1);

    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "Gagal mengonversi gambar dengan cv_bridge: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionNode>();

  rclcpp::spin(node);
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}