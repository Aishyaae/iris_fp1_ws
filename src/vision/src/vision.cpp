#include <memory>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64.hpp"

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

    // 1. Inisialisasi Publisher untuk mengirim sudut ke package master
    pub_angle = create_publisher<std_msgs::msg::Float64>("/vision/lane_angle", 10);

    // Subscribe ke topic kamera depan
    sub_image = create_subscription<sensor_msgs::msg::Image>(
      "/catvehicle/camera_front/image_raw_front",
      qos_profile,
      std::bind(&VisionNode::image_callback, this, std::placeholders::_1)
    );
  }

private:

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      
      // Konversi dari ROS Image Message ke OpenCV Matrix (BGR8)
      Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

      Mat frame_bev, frame_hsv, frame_threshold, frame_output;
      
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

      cvtColor(frame_bev, frame_hsv, COLOR_BGR2HSV);

      // thereshold
      inRange(frame_hsv, Scalar(h[0], s[0], v[0]), Scalar(h[1], s[1], v[1]), frame_threshold);

      // contour
      vector<vector<Point>> contours;
      vector<Vec4i> hierarcy;
      findContours(frame_threshold, contours, hierarcy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

      double final_angle = 0.0;
      Point target_point(width / 2, height - 100); // default lurus ke depan 
      double biggest_area = 0.0;

      Mat frame_angle = frame_bev.clone();

      // filter jika objek terlalu kecil
      for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if(area < 150) continue;

        // hitung centroid
        Moments m = moments(contours[i]);
        if (m.m00 != 0) {
          double cx = (m.m10 / m.m00);
          double cy = (m.m01 / m.m00);

          // titik tengah bawah gambar
          double dx = cx - (width / 2);
          double dy = height - cy;
          // kemiringan sudut
          double angle = atan2(dx, dy) * 180.0 / M_PI;

          // filter sudur toleransi <20 derajat
          if (abs(angle) < 20.0) {
            if (area > biggest_area) {
              biggest_area = area;
              target_point = Point(cx, cy); // koordinat
              final_angle = angle;
            }
          }
        }
      }

      if (biggest_area > 0.0) {
        line(frame_angle, Point(width / 2, height), target_point, Scalar(0, 255, 0), 3);
        circle(frame_angle, target_point, 7, Scalar(0, 0, 255), -1);
      }

      // publish final_angle ke topic
      std_msgs::msg::Float64 angle_msg;
      angle_msg.data = final_angle;
      pub_angle->publish(angle_msg);

      // if(!contours.empty()) {
      //   printf("Ada contour");
      // }

      imshow("RAW", frame);
      imshow("Thresholding", frame_threshold);
      imshow("BEV", frame_angle);
      
      waitKey(1);

    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "Gagal mengonversi gambar dengan cv_bridge: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_angle;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = make_shared<VisionNode>();

  rclcpp::spin(node);
  destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}