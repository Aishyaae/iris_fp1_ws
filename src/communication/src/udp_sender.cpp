// #include <arpa/inet.h> // konversi alamat htons, inet_pton
// #include <endian.h> // htole32
// #include <netinet/in.h> // definisiin struktur alamat
// #include <sys/socket.h> // fungsi socket() sendto()
// #include <unistd.h> // close() nutup socket

// #include <algorithm> // clamp, max
// #include <chrono> // satuan waktu pada timer
// #include <cstdint> // tipe integer ukuran pasti int32_t, uint8_t, uint32_t
// #include <cstring> // memcpy
// #include <memory> // make_shared untuk buat node
// #include <stdexcept> // runtime_error kalo ada socket gagal dibuat
// #include <string> // string untuk parameter host

// #include "rclcpp/rclcpp.hpp" //buat node, publisher, subscriber, timer
// #include "std_msgs/msg/int32_multi_array.hpp" // tipe pesan ros2 berupa array integer 32 bit untuk format perintah

// // membuat kelas UdpSender yang mewarisi kelas dasar rclcpp::Node
// class UdpSenderNode : public rclcpp::Node {
// public:
//   UdpSenderNode() : Node("udp_sender_node") {
//     // constructor dipanggil saat objek dibuat. memberi nama node udp_sender_node


//     // declare parameter ros. kalo gak diubah dari luar, dipakai nilai default
//     host_ = declare_parameter<std::string>("host", "127.0.0.1"); // IP tujuan
//     port_ = declare_parameter<int>("port", 12151); // port UDP simulasi
//     rate_hz_ = declare_parameter<int>("rate_hz", 20); // seberapa sering paket dikirim per detik
//     timeout_ms_ = declare_parameter<int>("cmd_timeout_ms", 500); // batas waktu tanpa perintah baru sebelum mobil dihentikan

//     setup_socket(); // memanggil fungsi private untuk membuat socket dan menyiapkan alamat tujuan. harus setelah parameter terbaca karena butuh host_ dan port_

//     // subscriber: data[0] = pwm, data[1] = steer (derajat). buat subcriber
//     sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
//         "/cmd_motor", 10, // nama topik yg didengarkan, 10 ukuran antrean (kalo 10 pesan menumpuk, yang paling lama dibuang)
//         std::bind(&UdpSenderNode::cmd_callback, this, std::placeholders::_1)); // hubungin topic ke cmd_callback, this menunjuk objek ini, _1 adalah tempat pesan masuk

//     // timer pengiriman yang manggil timer_callback secara periodik
//     timer_ = create_wall_timer( // wall_timer jam dinding
//         std::chrono::milliseconds(1000 / std::max(1, rate_hz_)), // ubah frekuensi jadi periode ms (20hz -> 50ms)
//         // std::max cegah pembagian dengan 0 kalau rate 0
//         std::bind(&UdpSenderNode::timer_callback, this));

//     last_cmd_time_ = now(); // catat waktu sekarang sebagai "terakhir terima perintah". untuk hitungan timeout
//     RCLCPP_INFO(get_logger(), "Sending UDP to %s:%d at %d Hz", host_.c_str(), port_, rate_hz_); // cetak info ke terminal
//     // c_str
//   }

//   // dipanggil saat node dihancurkan (Ctrl+C / shutdown)
//   ~UdpSenderNode() override {
//     if (sock_ >= 0) { // hanya valid kalo socket jadi
//       pwm_ = 0;  // paket stop, steering tetap
//       send_packet(); // kirim paket terakhir
//       RCLCPP_INFO(get_logger(), "Stopped"); // cetak log
//       close(sock_); // tutup socket
//     }
//   }

// private:
//   static constexpr char kHeader[4] = {'I', 'R', 'I', 'S'}; // static artinya satu salinan dipakai di semua objek
//   // constextpr nilainya tetap dan sdh ditentukan saat kompilasi. simulasi hanya nerima paket yg diawali "iris"
//   static constexpr size_t kPacketSize = 12; // ukuran paket

//   // membuat socket UDP dan menyiapkan alamat tujuan
//   void setup_socket() {
//     sock_ = socket(AF_INET, SOCK_DGRAM, 0); //membuat socket. SOCK_DIAGRAM (udp), 0 protokol default
//     // hasilnya berupa file descriptor (angka). nilai negatif berarti gagal
//     if (sock_ < 0) {
//       throw std::runtime_error("Failed to create socket"); // kalo gagal, lempar exception pesan error
//     }

//     addr_ = {}; // kosongin struct alamat agar gaada sisa data acak
//     addr_.sin_family = AF_INET;
//     addr_.sin_port = htons(static_cast<uint16_t>(port_));
//     if (inet_pton(AF_INET, host_.c_str(), &addr_.sin_addr) != 1) { // ubah ip teks jadi 4 byte biner di sin_addr dan return 1 kalo berhasil
//       close(sock_); // kalo ip gk valid, tutup socket
//       sock_ = -1; // tandai
//       throw std::runtime_error("Invalid host: " + host_); // lempar error
//     }
//   }

//   // isi buffer 12 byte: "IRIS" | pwm (int32 LE) | steer (int32 LE)
//   void build_packet(uint8_t (&buf)[kPacketSize], int32_t pwm, int32_t steer) {
//     // membatasi nilai rentang untuk simulasi
//     pwm = std::clamp<int32_t>(pwm, -255, 255);
//     steer = std::clamp<int32_t>(steer, -90, 90);
//     // static_cast mengubah int bertanda jadi tak bertanda dengan pola bit yg sama, karena htole bekerja pada uint32_t
//     // htole32 memastikan urutan byte little endian di mesin apapun
//     uint32_t pwm_le = htole32(static_cast<uint32_t>(pwm));
//     uint32_t steer_le = htole32(static_cast<uint32_t>(steer));

//     // menyusun paket byte demi byte
//     std::memcpy(buf, kHeader, sizeof(kHeader));
//     std::memcpy(buf + 4, &pwm_le, sizeof(pwm_le));
//     std::memcpy(buf + 8, &steer_le, sizeof(steer_le));
//   }

//   void send_packet() {
//     // membuat buffer lokal lalu isi dari nilai terbaru yg tersimpan
//     uint8_t packet[kPacketSize];
//     build_packet(packet, pwm_, steer_);
//     // mengirim data (socket yg dipakai, data, panjangnya, ..., ukuran struct alamat, hasil negatif berarti gagal catat jadi error)
//     if (sendto(sock_, packet, sizeof(packet), 0, reinterpret_cast<sockaddr *>(&addr_), sizeof(addr_)) < 0) {
//       RCLCPP_ERROR(get_logger(), "sendto failed");
//     }
//   }

//   // simpan perintah terbaru dari topic
//   // msg pointer ke pesan yg masuk
//   void cmd_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
//     if (msg->data.size() < 2) { // array harus ada min 2 elemen
//       RCLCPP_WARN(get_logger(), "cmd_motor needs 2 values: [pwm, steer]"); // kalo kurang, beri pesan dan abaikan
//       return;
//     }
//     // menyimpan perintah terbaru
//     pwm_ = msg->data[0]; // index 0 pwm
//     steer_ = msg->data[1]; // index 1 steer
//     last_cmd_time_ = now(); // memperbarui waktu terakhir ada perintah
//   }

//   // dipanggil periodik oleh timer: kirim perintah terakhir ke simulasi
//   void timer_callback() {
//     // pengaman: tidak ada perintah baru -> berhenti
//     if ((now() - last_cmd_time_).nanoseconds() > timeout_ms_ * 1000000LL) {
//       pwm_ = 0;
//     }
//     send_packet(); // kirim paket terakhir
//   }

//   // --- member ---
//   int sock_ = -1;
//   sockaddr_in addr_{};
//   std::string host_;
//   int port_;
//   int rate_hz_;
//   int timeout_ms_;

//   int32_t pwm_ = 0;
//   int32_t steer_ = 0;
//   rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

//   rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
//   rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   // buat node dan spin jalankan event loop menunggu pesan dan timer sampai ctrl+c
//   // setelah spin selesai, node dihancurkan dan destructor memanggil paket stop
//   rclcpp::spin(std::make_shared<UdpSenderNode>());
//   rclcpp::shutdown(); // membersihkan resource ros, lalu program selesai
//   return 0;
// }



// teleop awsd
#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class TeleopUdpNode : public rclcpp::Node {
public:
  TeleopUdpNode() : Node("teleop_udp_node") {
    host_ = declare_parameter<std::string>("host", "127.0.0.1");
    port_ = declare_parameter<int>("port", 12151);
    pwm_step_ = declare_parameter<int>("pwm_step", 150);
    steer_step_ = declare_parameter<int>("steer_step", 10);
    max_pwm_ = std::clamp<int>(declare_parameter<int>("max_pwm", 150), 0, 255);
    max_steer_ = std::clamp<int>(declare_parameter<int>("max_steer", 45), 0, 90);

    setup_socket();
    enable_raw_mode();

    // Publisher mengirim target tombol WASD ke Master PID
    pub_target_ = create_publisher<std_msgs::msg::Int32MultiArray>("/cmd_target", 10);

    // Subscriber menerima nilai halus dari Master PID
    sub_smooth_ = create_subscription<std_msgs::msg::Int32MultiArray>(
    "/cmd_smooth_motor", 10,
        std::bind(&TeleopUdpNode::smooth_callback, this, std::placeholders::_1));

    // Timer 20 Hz (50 ms): baca keyboard + kirim paket UDP langsung ke Gazebo
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&TeleopUdpNode::on_timer, this));

    std::printf("\n=== Teleop WASD (UDP Langsung + Terhubung PID Master) ===\n"
    "W/S: Target PWM +/-  | A/D: Target Steer Kiri/Kanan\n"
    "Spasi: Stop          | X: Steer Lurus | Q: Keluar\n\n");
  }

  ~TeleopUdpNode() override {
    smooth_pwm_ = 0; // Stop motor
    send_packet();
    restore_terminal();
    if (sock_ >= 0) close(sock_);
    std::printf("\nStopped\n");
  }

private:
  static constexpr char kHeader[4] = {'I', 'R', 'I', 'S'};
  static constexpr size_t kPacketSize = 12;

  void setup_socket() {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) throw std::runtime_error("Failed to create socket");

    addr_ = {};
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(static_cast<uint16_t>(port_));
    if (inet_pton(AF_INET, host_.c_str(), &addr_.sin_addr) != 1) {
      close(sock_);
      sock_ = -1;
      throw std::runtime_error("Invalid host: " + host_);
    }
  }

  // Mengirim paket UDP 12-byte langsung ke Port 12151 Gazebo
  void send_packet() {
    if (sock_ < 0) return;
    uint8_t buf[kPacketSize];

    uint32_t pwm_le = htole32(static_cast<uint32_t>(smooth_pwm_));
    uint32_t steer_le = htole32(static_cast<uint32_t>(smooth_steer_));

    std::memcpy(buf, kHeader, 4);
    std::memcpy(buf + 4, &pwm_le, 4);
    std::memcpy(buf + 8, &steer_le, 4);

    sendto(sock_, buf, sizeof(buf), 0,
           reinterpret_cast<sockaddr *>(&addr_), sizeof(addr_));
  }

  void enable_raw_mode() {
    tcgetattr(STDIN_FILENO, &old_tio_);
    termios raw = old_tio_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0; // Non-blocking read
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    raw_enabled_ = true;
  }

  void restore_terminal() {
    if (raw_enabled_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
      raw_enabled_ = false;
    }
  }

  void handle_key(char c) {
    switch (std::tolower(static_cast<unsigned char>(c))) {
      case 'w': target_pwm_ += pwm_step_; break;
      case 's': target_pwm_ -= pwm_step_; break;
      case 'a': target_steer_ -= steer_step_; break;
      case 'd': target_steer_ += steer_step_; break;
      case ' ': target_pwm_ = 0; break;
      case 'x': target_steer_ = 0; break;
      case 'q': rclcpp::shutdown(); return;
      default: return;
    }
    target_pwm_ = std::clamp(target_pwm_, -max_pwm_, max_pwm_);
    target_steer_ = std::clamp(target_steer_, -max_steer_, max_steer_);

    // Publish target baru ke Node Master
    std_msgs::msg::Int32MultiArray target_msg;
    target_msg.data = {target_pwm_, target_steer_};
    pub_target_->publish(target_msg);
  }

  // Menerima nilai PWM & Steer halus hasil olahan PID dari Node Master
  void smooth_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 2) return;
    smooth_pwm_ = msg->data[0];
    smooth_steer_ = msg->data[1];
  }

  void on_timer() {
    char c;
    while (read(STDIN_FILENO, &c, 1) == 1) handle_key(c);

    // Kirim data hasil PID langsung ke Gazebo via UDP
    send_packet();

    std::printf("\rTarget: [PWM=%4d STEER=%4d] | Hasil PID Gazebo: [PWM=%4d STEER=%4d]   ",
                target_pwm_, target_steer_, smooth_pwm_, smooth_steer_);
    std::fflush(stdout);
  }

  int sock_ = -1;
  sockaddr_in addr_{};
  std::string host_;
  int port_;
  int pwm_step_, steer_step_, max_pwm_, max_steer_;

  int target_pwm_ = 0, target_steer_ = 0;
  int smooth_pwm_ = 0, smooth_steer_ = 0;

  termios old_tio_{};
  bool raw_enabled_ = false;

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_target_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_smooth_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopUdpNode>());
  rclcpp::shutdown();
  return 0;
}