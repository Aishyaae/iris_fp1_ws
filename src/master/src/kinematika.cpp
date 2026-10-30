#include <memory>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std;

class MasterNode : public rclcpp::Node {
public:
  MasterNode() : Node("master_node") {
    // subscribe ke data dari package vision
    sub_angle = create_subscription<std_msgs::msg::Float64>(
      "/vision/lane_angle",
      10,
      bind(&MasterNode::angle_callback, this, placeholders::_1)
    );

    // publisher untuk mengirim hasil kalkulasi ke package communication
    pub_control = create_publisher<std_msgs::msg::Int32MultiArray>("/master/geometry_control", 10);
    
    RCLCPP_INFO(this->get_logger(), "Master Node telah aktif dan mendengarkan data dari Vision.");
  }

private:
    double kp = 1.5;
    double ki = 0.05;
    double kd = 0.4;
    double kff = 1.0; // feedforward

    double last_error = 0.0;
    double integral_error = 0.0;
    double dt = 0.05;

  void angle_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    double actual_angle = msg->data; // Sudut dalam derajat dari vision (-20 sampai +20)
    double target_angle = 0.0; // lurus di tengah jalan

    // hitung error
    double error = target_angle - actual_angle;

    double angle_rad = error * M_PI / 180.0;

    // hitung proportional
    double P = kp * sin(angle_rad);

    // hitung integral
    integral_error += error * dt;
    double I = ki * integral_error;

    // hitung derivative
    double derivative = (error - last_error) / dt;
    double D = kd * derivative;

    double u_pid = P + I + D; // gabungan pid
    double u_ff = kff * sin(angle_rad); // feedforward

    double u_total = u_ff + u_pid; // instruksi daya final

    int32_t steering_degree = static_cast<int32_t>(u_total * 45.0);
    int32_t motor_pwm = static_cast<int32_t>(180.0 * cos(actual_angle * M_PI / 180.0));

    last_error = error;

    // clamp batas
    steering_degree = clamp<int32_t>(steering_degree, -90, 90);
    motor_pwm = clamp<int32_t>(motor_pwm, -255, 255);

    // kemas data
    std_msgs::msg::Int32MultiArray control_msg;
    control_msg.data.clear();
    control_msg.data.push_back(motor_pwm); // indeks[0] = pwm
    control_msg.data.push_back(steering_degree); // indeks[1] = steering degree

    pub_control->publish(control_msg);

    RCLCPP_INFO(this->get_logger(), "Publish Int -> PWM: %d | Setir: %d deg", motor_pwm, steering_degree);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_angle;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_control;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = make_shared<MasterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
