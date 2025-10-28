#include "SerialPort.h"
#include <algorithm>
#include <cstring>

namespace HSCanT {
#include <glob.h>

// 串口节点前缀，使用四路can都是这个前缀
constexpr char SERIAL_PREFIX[] =
    "/dev/serial/by-id/usb-HPMicro_USB_Virtual_COM*";

constexpr char hex2char[] = "0123456789ABCDEF";

// 数据发送为ascii格式，每个字节代表一位ascii字符
struct hscan_command_t {
  uint8_t command;  // 发送命令 't'
  uint8_t canId[3]; // 电机id
  uint8_t len;      // len '8'
  uint8_t data[16]; // 8位数据，每个字节用两个ascii字符表示
  uint8_t CR;       // 命令终止 '\r'
};

inline uint8_t hexChar2Nibble(uint8_t c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return 10 + (c - 'A');
  if (c >= 'a' && c <= 'f')
    return 10 + (c - 'a');
  // 无效字符
  return 0;
}

class HSCanT_handler {
public:
  HSCanT_handler(std::unique_ptr<Serial::SerialPort> &sptr)
      : serial_ptr_(std::move(sptr)) {
    if (serial_ptr_ == nullptr || !serial_ptr_->is_Open()) {
      LOGE("HSCanT_handler serial port is not valid");
    }
  }
  ~HSCanT_handler() {
    if (serial_ptr_->is_Open()) {
      serial_ptr_->close();
    }
  }

  // 这里只发标准帧，扩展帧后续再加
  void send_frame(uint32_t id, uint8_t *motor_frame, int size) {
    hscan_command_t send_buf;
    send_buf.command = 't';
    // 默认size = 8
    send_buf.len = '8';
    send_buf.CR = '\r';

    if (size != 8) {
      LOGW("HSCanT send frame size mismatch");
      return;
    }
    // 发送电机id和数据
    send_buf.canId[0] = '0' + (id / 100);
    send_buf.canId[1] = '0' + ((id / 10) % 10);
    send_buf.canId[2] = '0' + (id % 10);
    for (int i = 0; i < 8; i++) {
      uint8_t byte = motor_frame[i];
      send_buf.data[2 * i] = hex2char[byte >> 4];       // 高4位
      send_buf.data[2 * i + 1] = hex2char[byte & 0x0F]; // 低4位
    }
    serial_ptr_->send((uint8_t *)&send_buf, sizeof(hscan_command_t));
  }

  void recv_frame(uint32_t &id, uint8_t *data_buf, size_t buf_size) {
    hscan_command_t recv_cmd;
    serial_ptr_->recv((uint8_t *)&recv_cmd, sizeof(recv_cmd));
    // 检查命令头和结束符，这里默认为标准can帧
    // ! 改写扩展帧时注意这里
    if (recv_cmd.command != 't' || recv_cmd.len != '8' || recv_cmd.CR != '\r') {
      LOGW("HSCanT receive frame error");
      return;
    }

    std::array<uint8_t, 8> data;
    for (int i = 0; i < 8; i++) {
      uint8_t highChar = recv_cmd.data[2 * i];
      uint8_t lowChar = recv_cmd.data[2 * i + 1];

      uint8_t highNibble = hexChar2Nibble(highChar);
      uint8_t lowNibble = hexChar2Nibble(lowChar);

      data[i] = (highNibble << 4) | lowNibble;
    }
    if (buf_size != data.size()) {
      LOGW("HSCanT receive data buffer size mismatch");
    }
    memcpy(data_buf, data.data(), std::min(buf_size, data.size()));
    id = (uint32_t(recv_cmd.canId[0] - '0') * 100) +
         (uint32_t(recv_cmd.canId[1] - '0') * 10) +
         uint32_t(recv_cmd.canId[2] - '0');
  }

private:
  std::unique_ptr<Serial::SerialPort> serial_ptr_;
};

// HSCanT 是 HSCanT_handler 的工厂类
class HSCanT {
public:
  HSCanT() {
    find_serial_ports_();
    // 四路can一定要有四个节点
    if (serial_devices_.empty() || serial_devices_.size() < 4) {
      valid_ = false;
      return;
    }

    for (const auto &device : serial_devices_) {
      auto serial_ptr = std::make_unique<Serial::SerialPort>(device, B921600);
      serial_ptrs_.push_back(std::move(serial_ptr));

      if (!serial_ptrs_.back()->is_Open()) {
        LOGE("Failed to open serial port %s", device.c_str());
        valid_ = false;
        return;
      }
    }

    for (int i = 0; i < serial_ptrs_.size(); i++) {
      // 使用索引初始化HPM端口名
      if (!hscant_start_transmit(serial_ptrs_[i], '0' + i)) {
        LOGE("Failed to start transmit on port %s", serial_devices_[i].c_str());
        valid_ = false;
        return;
      }
    }
    valid_ = true;

    assert(valid_);
  }

  ~HSCanT() {
    // 此类已经没有串口指针的所有权，应从HSCanT_handler销毁串口
    LOGI("HSCanT Exited!!");
  }

  // 此处移交了串口指针的所有权
  std::unique_ptr<HSCanT_handler> export_handler(int serial_index) {
    if (valid_ == false || serial_index < 0 ||
        serial_index >= serial_ptrs_.size() ||
        serial_ptrs_[serial_index] == nullptr) {
      LOGE("Export HSCanT ptr Error");
      return nullptr;
    }
    return std::make_unique<HSCanT_handler>(serial_ptrs_[serial_index]);
  }

  bool is_valid() const { return valid_; }

private:
  void find_serial_ports_() {
    // 使用glob查找所有匹配的串口设备
    glob_t glob_result;
    int ret = glob(SERIAL_PREFIX, GLOB_TILDE, NULL, &glob_result);
    if (ret == 0) {
      for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
        serial_devices_.push_back(glob_result.gl_pathv[i]);
        LOGI("Find available device: %s", glob_result.gl_pathv[i]);
      }
    } else {
      LOGE("No serial devices found with prefix %s", SERIAL_PREFIX);
    }

    // 按照最后一位数字字符排序，保证初始化顺序
    std::sort(serial_devices_.begin(), serial_devices_.end(),
              [](const std::string &a, const std::string &b) {
                return a.back() < b.back();
              });
    globfree(&glob_result);
  }

  bool hscant_start_transmit(std::unique_ptr<Serial::SerialPort> &sptr,
                             char can_port) {
    uint8_t recv_buf[13];
    // 防止打印越界
    recv_buf[12] = '\0';
    // 这里初始化七个字符，含有\0方便打印，但串口只发6字节
    uint8_t can_name[7] = "r_can0";
    can_name[5] = can_port;

    sptr->send((uint8_t *)can_name, 6);
    // 接收返回字符串判断usb通信是否正常，一般回复HPM_CANx_BUS共12字节
    int reply_len = sptr->recv(recv_buf, 12);
    if (reply_len != 12) {
      LOGE("CAN Port didnt reply");
      return false;
    }
    LOGI("PORT %s Initialized", recv_buf);

    // S8\r 选择波特率为1000000
    sptr->send((uint8_t *)"S8\r", 3);
    usleep(200);
    // usb acm 115200 波特率
    sptr->send((uint8_t *)"U1\r", 3);
    usleep(200);
    // 打开
    sptr->send((uint8_t *)"O\r", 2);
    usleep(200);
    return true;
  }

private:
  std::vector<std::unique_ptr<Serial::SerialPort>> serial_ptrs_;
  std::vector<std::string> serial_devices_;
  bool valid_;
};

}; // namespace HSCanT