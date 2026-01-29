#ifndef DAMIAO_H
#define DAMIAO_H

#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <unistd.h>
#include <functional>
#include <initializer_list>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#define POS_MODE 0x100
#define SPEED_MODE 0x200
#define POSI_MODE 0x300
#define max_retries 20
#define retry_interval 50000

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

namespace damiao {

#pragma pack(1)
#define Motor_id uint32_t

/*!
 * @brief Motor Type 电机类型
 */
enum MotorType {
  DM4310,
  DM4310_48V,
  DM4340,
  DM4340_48V,
  DM6006,
  DM8006,
  DM8009,
  DM10010L,
  DM10010,
  DMH3510,
  DMH6215,
  DMG6220,
  RS00,
  RS01,
  RS02,
  RS03,
  RS04,
  RS05,
  RS06,
  Num_Of_Motor
};

typedef struct {
  uint8_t FrameHeader;
  uint8_t CMD;  // 命令 0x00: 心跳
  //     0x01: receive fail 0x11: receive success
  //     0x02: send fail 0x12: send success
  //     0x03: set baudrate fail 0x13: set baudrate success
  //     0xEE: communication error 此时格式段为错误码
  //     8: 超压 9: 欠压 A: 过流 B: MOS过温 C: 电机线圈过温 D: 通讯丢失 E: 过载
  uint8_t canDataLen : 6;  // 数据长度
  uint8_t canIde : 1;      // 0: 标准帧 1: 扩展帧
  uint8_t canRtr : 1;      // 0: 数据帧 1: 远程帧
  uint32_t canId;          // 电机反馈的ID
  uint8_t canData[8];
  uint8_t frameEnd;  // 帧尾
} Can_Receive_Frame;

typedef struct {
  uint8_t FrameHeader[2] = {0x55, 0xAA};  // 帧头
  uint8_t FrameLen = 0x1e;                // 帧长
  uint8_t CMD = 0x03;  // 命令 1：转发CAN数据帧 2：PC与设备握手，设备反馈OK 3:
                       // 非反馈CAN转发，不反馈发送状态
  uint32_t sendTimes = 1;      // 发送次数
  uint32_t timeInterval = 10;  // 时间间隔
  uint8_t IDType = 0;          // ID类型 0：标准帧 1：扩展帧
  uint32_t canId = 0x01;       // CAN ID 使用电机ID作为CAN ID
  uint8_t frameType = 0;       // 帧类型 0： 数据帧 1：远程帧
  uint8_t len = 0x08;          // len
  uint8_t idAcc = 0;
  uint8_t dataAcc = 0;
  uint8_t data[8] = {0};
  uint8_t crc = 0;  // 未解析，任意值

  void modify(const Motor_id id, const uint8_t *send_data) {
    canId = id;
    std::copy(send_data, send_data + 8, data);
  }

} Can_Send_Frame;

#pragma pack()
typedef struct {
  float Q_MAX;
  float DQ_MAX;
  float TAU_MAX;
  float KP_MAX;
  float KD_MAX;
} Limit_param;

//电机PMAX DQMAX TAUMAX参数
extern Limit_param limit_param[Num_Of_Motor];

struct ActData {
  std::string name;
  MotorType motorType;
  int can_id;
  int mst_id;
  double pos, vel, effort;
  double cmd_pos, cmd_vel, cmd_effort;
  double kp, kd;
};

class Motor {
 private:
  /* data */
  Motor_id Master_id;
  Motor_id Slave_id;
  float state_q = 0;
  float state_dq = 0;
  float state_tau = 0;
  Limit_param limit_param{};
  MotorType motor_type;

  union ValueUnion {
    float floatValue;
    uint32_t uint32Value;
  };

  struct ValueType {
    ValueUnion value;
    bool isFloat;
  };

  std::unordered_map<uint32_t, ValueType> param_map;

 public:
  Motor(MotorType motor_type, Motor_id Slave_id, Motor_id Master_id);
  void receive_data(float q, float dq, float tau);
  MotorType GetMotorType() const { return this->motor_type; }
  Motor_id GetMasterId() const { return this->Master_id; }
  Motor_id GetSlaveId() const { return this->Slave_id; }
  float Get_Position() const { return this->state_q; }
  float Get_Velocity() const { return this->state_dq; }
  float Get_tau() const { return this->state_tau; }
  Limit_param get_limit_param() { return limit_param; }
};

// TODO(me): 实现电机意外断开连接的自动恢复机制
// TODO(me): 可以试试链表实现的无锁队列
template <typename T>
class Queue {
 private:
  mutable std::mutex mutex_;
  std::queue<T> queue_;
  std::condition_variable cond_;
  bool shutdown_;

 public:
  Queue() : shutdown_(false){};
  ~Queue() = default;

  Queue(const Queue &) = delete;
  Queue &operator=(const Queue &) = delete;

  bool try_push(T value) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (shutdown_) {
      return false;
    }

    queue_.push(std::move(value));
    cond_.notify_one();
    return true;
  };
  bool try_pop(T &value) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (queue_.empty() || shutdown_) {
      return false;
    }

    value = std::move(queue_.front());
    queue_.pop();
    return true;
  };

  bool pop(T &value) {
    std::unique_lock<std::mutex> lock(mutex_);

    cond_.wait(lock, [this]() { return !queue_.empty() || shutdown_; });

    if (shutdown_ && queue_.empty()) {
      return false;
    }

    value = std::move(queue_.front());
    queue_.pop();
    return true;
  };
  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  };
  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  };
  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    while (!queue_.empty()) {
      queue_.pop();
    }
  };
  void shutdown() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutdown_ = true;
    }
    cond_.notify_all();
  };
  bool is_shutdown() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shutdown_;
  };
};

class Motor_Control {
 public:
  Motor_Control(void *hscant_handler);
  ~Motor_Control();
  void get_motor_data();
  void enable();
  void write();
  void read();
  void refresh_motor_status(const Motor &motor);
  void refresh_allmotor_status();
  void disable();
  void set_zero_position();
  void control_mit(Motor &DM_Motor, float kp, float kd, float q, float dq,
                   float tau);
  void control_pos_vel(Motor &DM_Motor, float pos, float vel);
  void control_vel(Motor &DM_Motor, float vel);
  void receive();
  void addMotor(std::initializer_list<std::shared_ptr<Motor>> Motor_list);
  static void changeMotorLimit(Motor &DM_Motor, float P_MAX, float Q_MAX,
                               float T_MAX);

 private:
  void control_cmd(Motor_id id, uint8_t cmd);
  void update_motor();

  static bool is_in_ranges(int number) {
    return (7 <= number && number <= 10) || (13 <= number && number <= 16) ||
           (35 <= number && number <= 36);
  }

  static uint32_t float_to_uint32(float value) {
    return static_cast<uint32_t>(value);
  }

  static float uint32_to_float(uint32_t value) {
    return static_cast<float>(value);
  }

  static float uint8_to_float(const uint8_t data[4]) {
    uint32_t combined = (static_cast<uint32_t>(data[3]) << 24) |
                        (static_cast<uint32_t>(data[2]) << 16) |
                        (static_cast<uint32_t>(data[1]) << 8) |
                        static_cast<uint32_t>(data[0]);
    float result;
    memcpy(&result, &combined, sizeof(result));
    return result;
  }
  std::thread update_thread;
  std::atomic<bool> stop_update_thread_;

  std::unordered_map<Motor_id, std::shared_ptr<Motor>> motors;
  void *hscant_handler;
  Queue<Can_Send_Frame> send_queue;

  // ! 似乎是一个由用户维护的数据，用于集中管理单个串口的多个电机
  // ! 配套的api有MotorContorl构造函数、write、read函数
  // TODO(me): 暂时不用像这样暴露整个数据，后续修改
  std::unordered_map<Motor_id, ActData> act_data;
};

};  // namespace damiao

#endif
