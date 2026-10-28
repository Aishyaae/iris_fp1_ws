#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

// Kelas Controller PID
class PIDController {
public:
  PIDController(double kp, double ki, double kd)
      : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

  double compute(double setpoint, double actual, double dt) {
    if (dt <= 0.0) return 0.0;
    double error = setpoint - actual;
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -10.0, 10.0); // Anti-windup
    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;
    return (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
  }

  void reset() {
    prev_error_ = 0.0;
    integral_ = 0.0;
  }

private:
  double kp_, ki_, kd_;
  double prev_error_;
  double integral_;
};

class SteeringControllerNode : public rclcpp::Node {
public:
  SteeringControllerNode() : Node("kinematika_node"), pid_(3.0, 0.01, 0.1) {
    wheelbase_ = declare_parameter<double>("wheelbase", 2.62); // Wheelbase 2.62 m

    // Subscriber menerima target pergerakan dari Teleop
    sub_target_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        "/cmd_target", 10,
        std::bind(&SteeringControllerNode::target_callback, this, std::placeholders::_1));

    // Publisher mengirimkan hasil sudut halus PID balik ke Teleop
    pub_smooth_ = create_publisher<std_msgs::msg::Int32MultiArray>("/cmd_smooth_motor", 10);

    // Timer loop PID 20 Hz (50 ms)
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&SteeringControllerNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "Master PID Controller Aktif (Wheelbase: %.2f m)", wheelbase_);
  }

private:
  void target_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 2) return;
    target_pwm_ = msg->data[0];
    target_steer_ = static_cast<double>(msg->data[1]);
  }

  void control_loop() {
    double dt = 0.05; // 50 ms (20 Hz)

    // Hitung PID untuk memperhalus pergerakan sudut belok
    double steer_output = pid_.compute(target_steer_, actual_steer_, dt);
    actual_steer_ += steer_output * dt;
    actual_steer_ = std::clamp(actual_steer_, -45.0, 45.0);

    // Kirim nilai PWM dan Steer hasil PID ke Teleop
    std_msgs::msg::Int32MultiArray smooth_msg;
    smooth_msg.data = {target_pwm_, static_cast<int32_t>(std::round(actual_steer_))};
    pub_smooth_->publish(smooth_msg);
  }

  double wheelbase_;
  int32_t target_pwm_ = 0;
  double target_steer_ = 0.0;
  double actual_steer_ = 0.0;

  PIDController pid_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_target_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_smooth_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringControllerNode>());
  rclcpp::shutdown();
  return 0;
}