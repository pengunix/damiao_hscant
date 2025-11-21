#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H
#include <array>
#include <chrono>
#include <cstring>
#include <memory>
#include <queue>
#include <string>
#include <unistd.h>

#ifdef USE_ROS
#include <ros/ros.h>
#define LOGD(...) ROS_DEBUG(__VA_ARGS__)
#define LOGI(...) ROS_INFO(__VA_ARGS__)
#define LOGW(...) ROS_WARN(__VA_ARGS__)
#define LOGE(...) ROS_ERROR(__VA_ARGS__)
#else
#define LOGD(...)                                                              \
  printf(__VA_ARGS__);                                                         \
  printf("\n")
#define LOGI(...)                                                              \
  printf(__VA_ARGS__);                                                         \
  printf("\n")
#define LOGW(...)                                                              \
  printf(__VA_ARGS__);                                                         \
  printf("\n")
#define LOGE(...)                                                              \
  printf(__VA_ARGS__);                                                         \
  printf("\n")
#endif

namespace Serial {

extern "C" {
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
}

class SerialPort {
public:
  using SharedPtr = std::shared_ptr<SerialPort>;

  SerialPort(std::string port, speed_t baudrate, int timeout_ms = 5) {
    set_timeout(timeout_ms);
    Init(port, baudrate);
  }

  ~SerialPort() { close(); }

  ssize_t send(const uint8_t *data, size_t len) {
    // tcflush(fd_, TCIFLUSH);
    ssize_t ret = ::write(fd_, data, len);
    // tcdrain(fd_);
    return ret;
  }

  ssize_t recv(uint8_t *data, size_t len) {
    FD_ZERO(&rSet_);
    FD_SET(fd_, &rSet_);
    ssize_t recv_len = 0;

    switch (select(fd_ + 1, &rSet_, NULL, NULL, &timeout_)) {
    case -1: // error
      LOGW("[SerialPort] communication error");
      break;
    case 0: // timeout
      // Allow Timeout
      // LOGW("[SerialPort] Serial read timeout");
      break;
    default:
      recv_len = ::read(fd_, data, len);
      if (recv_len < 0) {
        LOGW("[SerialPort] read data error");
      }
      break;
    }
    return recv_len;
  }

  void recv(uint8_t *data, uint8_t head, ssize_t len) {
    // 存入队列
    ssize_t recv_len = this->recv(recv_buf.data(), len);
    for (int i = 0; i < recv_len; i++) {
      recv_queue.push(recv_buf[i]);
    }

    // 查找帧头
    while (recv_queue.size() >= len) {
      if (recv_queue.front() != head) {
        recv_queue.pop();
        continue;
      }
      break;
    }

    if (recv_queue.size() < len)
      return;

    // 读取数据
    for (int i = 0; i < len; i++) {
      data[i] = recv_queue.front();
      recv_queue.pop();
    }
  }

  void set_timeout(int timeout_ms) {
    timeout_.tv_sec = timeout_ms / 1000;
    timeout_.tv_usec = (timeout_ms % 1000) * 1000;
  }

  bool is_Open() const { return is_open; }

  void close() { ::close(fd_); }

private:
  void Init(std::string port, speed_t baudrate) {
    int ret;
    // Open serial port
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      LOGW("[SerialPort] Open serial port %s failed", port.c_str());
      is_open = false;
      exit(-1);
    }
    is_open = true;
    // Set attributes
    struct termios option;
    memset(&option, 0, sizeof(option));
    ret = tcgetattr(fd_, &option);

    option.c_oflag = 0;
    option.c_lflag = 0;
    option.c_iflag = 0;

    cfsetispeed(&option, baudrate);
    cfsetospeed(&option, baudrate);

    option.c_cflag &= ~CSIZE;
    option.c_cflag |= CS8;     // 8
    option.c_cflag &= ~PARENB; // no parity
    option.c_iflag &= ~INPCK;  // no parity
    option.c_cflag &= ~CSTOPB; // 1 stop bit
    // 关闭硬件流控
    option.c_cflag &= ~CRTSCTS;
    option.c_iflag &= ~(IXON | IXOFF | IXANY);

    option.c_cc[VTIME] = 0;
    option.c_cc[VMIN] = 0;
    option.c_lflag |= CBAUDEX;

    ret = tcflush(fd_, TCIFLUSH);
    ret = tcsetattr(fd_, TCSANOW, &option);
  }

  int fd_;
  fd_set rSet_;
  timeval timeout_;
  bool is_open;

  std::queue<uint8_t> recv_queue;
  std::array<uint8_t, 1024> recv_buf;
};

} // namespace Serial

#endif // SERIAL_PORT_H
