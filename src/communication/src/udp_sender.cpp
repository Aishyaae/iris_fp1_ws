#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <chrono>

using namespace std::chrono_literals;

class UdpSenderNode : public rclcpp::Node {
public:
    UdpSenderNode() : Node("udp_sender_node"), sock_(-1) {
        // Init socket UDP
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Gagal membuat socket UDP");
            return;
        }

        addr_.sin_family = AF_INET;
        addr_.sin_port = htons(12151); // Port UDP Sender sesuai spesifikasi
        inet_pton(AF_INET, "127.0.0.1", &addr_.sin_addr);

        // Subscriber untuk menerima perintah pergerakan dari package master
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&UdpSenderNode::cmdVelCallback, this, std::placeholders::_1)
        );

        // Timer pengiriman data 20 Hz (50 ms)
        timer_ = this->create_wall_timer(
            50ms, std::bind(&UdpSenderNode::sendUdpPacket, this)
        );

        RCLCPP_INFO(this->get_logger(), "UDP Sender Node siap mengirim ke port 12151");
    }

    ~UdpSenderNode() {
        if (sock_ >= 0) {
            // Send Stop Packet saat node ditutup (PWM = 0)
            uint8_t packet[12];
            buildPacket(packet, 0, current_steer_);
            sendto(sock_, packet, sizeof(packet), 0, reinterpret_cast<sockaddr*>(&addr_), sizeof(addr_));
            close(sock_);
        }
    }

private:
    int sock_;
    sockaddr_in addr_{};
    int32_t current_pwm_ = 0;
    int32_t current_steer_ = 0;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::TimerBase::SharedPtr timer_;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Contoh pemetaan sederhana linier.x -> PWM dan angular.z -> Steer
        current_pwm_ = static_cast<int32_t>(msg->linear.x);
        current_steer_ = static_cast<int32_t>(msg->angular.z);
    }

    void buildPacket(uint8_t (&buf)[12], int32_t pwm, int32_t steer) {
        // Clamping aman sesuai tabel modul
        pwm = std::clamp<int32_t>(pwm, -255, 255);
        steer = std::clamp<int32_t>(steer, -90, 90);

        uint32_t pwm_le = htole32(static_cast<uint32_t>(pwm));
        uint32_t steer_le = htole32(static_cast<uint32_t>(steer));

        const char header[4] = {'I', 'R', 'I', 'S'};
        std::memcpy(buf, header, 4);
        std::memcpy(buf + 4, &pwm_le, 4);
        std::memcpy(buf + 8, &steer_le, 4);
    }

    void sendUdpPacket() {
        uint8_t packet[12];
        buildPacket(packet, current_pwm_, current_steer_);
        sendto(sock_, packet, sizeof(packet), 0, reinterpret_cast<sockaddr*>(&addr_), sizeof(addr_));
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UdpSenderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}