#ifndef DAMIAO_H
#define DAMIAO_H

#include "SerialPort.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <thread>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>
#include <atomic>
#include <initializer_list>
#include <queue>
#include <mutex>
#include <condition_variable>


#define POS_MODE 0x100
#define SPEED_MODE 0x200
#define POSI_MODE 0x300
#define max_retries 20
#define retry_interval 50000

namespace damiao {

#pragma pack(1)
#define Motor_id uint32_t

/*!
 * @brief Motor Type 电机类型
 */
enum DM_Motor_Type {
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
  Num_Of_Motor
};

/*
 * @brief 电机控制模式
 */
enum Control_Mode {
  MIT_MODE = 1,
  POS_VEL_MODE = 2,
  VEL_MODE = 3,
  POS_FORCE_MODE = 4,
};

/*
 * @brief 寄存器列表 具体参考达妙手册
 */
enum DM_REG {
  UV_Value = 0,
  KT_Value = 1,
  OT_Value = 2,
  OC_Value = 3,
  ACC = 4,
  DEC = 5,
  MAX_SPD = 6,
  MST_ID = 7,
  ESC_ID = 8,
  TIMEOUT = 9,
  CTRL_MODE = 10,
  Damp = 11,
  Inertia = 12,
  hw_ver = 13,
  sw_ver = 14,
  SN = 15,
  NPP = 16,
  Rs = 17,
  LS = 18,
  Flux = 19,
  Gr = 20,
  PMAX = 21,
  VMAX = 22,
  TMAX = 23,
  I_BW = 24,
  KP_ASR = 25,
  KI_ASR = 26,
  KP_APR = 27,
  KI_APR = 28,
  OV_Value = 29,
  GREF = 30,
  Deta = 31,
  V_BW = 32,
  IQ_c1 = 33,
  VL_c1 = 34,
  can_br = 35,
  sub_ver = 36,
  u_off = 50,
  v_off = 51,
  k1 = 52,
  k2 = 53,
  m_off = 54,
  dir = 55,
  p_m = 80,
  xout = 81,
};

// TODO(me): ascii 解析不完全，丢失电机错误码信息，更改hscant固件后再来完善
#pragma pack(push, 1)
typedef struct {
  uint8_t FrameHeader;
  // uint8_t CMD;// 命令 0x00: 心跳
  //     0x01: receive fail 0x11: receive success
  //     0x02: send fail 0x12: send success
  //     0x03: set baudrate fail 0x13: set baudrate success
  //     0xEE: communication error 此时格式段为错误码
  //     8: 超压 9: 欠压 A: 过流 B: MOS过温 C: 电机线圈过温 D: 通讯丢失 E: 过载
  // uint8_t canDataLen: 6; // 数据长度
  // uint8_t canIde: 1; // 0: 标准帧 1: 扩展帧
  // uint8_t canRtr: 1; // 0: 数据帧 1: 远程帧
  uint8_t canId[3]; // 电机反馈的ID
  uint8_t len;      // len
  uint8_t canData[16];
  uint8_t frameEnd; // 帧尾
} CAN_Receive_Frame;
#pragma pack(pop)

// TODO(me)：发送改为透传模式
typedef struct can_send_frame {
  uint8_t FrameHeader = 0x74; // 帧头
  uint8_t canId[3] = {0};     // CAN ID 使用电机ID作为CAN ID
  uint8_t len = 0x38;         // len
  uint8_t data[16] = {0};
  uint8_t frameEnd = 0x0D;

  void modify(const Motor_id id, const uint8_t *send_data) {
    // canId = id;
    // std::copy(send_data, send_data+8, data);

    canId[0] = '0' + (id / 100);       // 百位
    canId[1] = '0' + ((id / 10) % 10); // 十位
    canId[2] = '0' + (id % 10);        // 个位
    const char hexChars[] = "0123456789ABCDEF";
    for (int i = 0; i < 8; i++) {
      uint8_t byte = send_data[i];
      data[2 * i] = hexChars[byte >> 4];       // 高4位
      data[2 * i + 1] = hexChars[byte & 0x0F]; // 低4位
    }
  }

} can_send_frame;

#pragma pack()

typedef struct {
  float Q_MAX;
  float DQ_MAX;
  float TAU_MAX;
} Limit_param;

//电机PMAX DQMAX TAUMAX参数
extern Limit_param limit_param[Num_Of_Motor];

struct DmActData {
  std::string name;
  DM_Motor_Type motorType;
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
  DM_Motor_Type Motor_Type;

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
  /**
   * @brief Construct a new Motor object
   *
   * @param Motor_Type 电机类型
   * @param Slave_id canId 从机ID即电机ID
   * @param Master_id 主机ID建议主机ID不要都设为0x00
   *
   */
  Motor(DM_Motor_Type Motor_Type, Motor_id Slave_id, Motor_id Master_id);

  void receive_data(float q, float dq, float tau);

  DM_Motor_Type GetMotorType() const { return this->Motor_Type; }

  /*
   * @brief get master id 获取主机ID
   * @return MasterID
   */
  Motor_id GetMasterId() const { return this->Master_id; }

  /*
   * @brief get motor slave id(can id)  获取电机CAN ID
   * @return SlaveID
   */
  Motor_id GetSlaveId() const { return this->Slave_id; }

  float Get_Position() const { return this->state_q; }

  float Get_Velocity() const { return this->state_dq; }

  float Get_tau() const { return this->state_tau; }

  /*
   * @brief get limit param 获取电机限制参数
   * @return limit_param 电机限制参数
   */
  Limit_param get_limit_param() { return limit_param; }

  void set_param(int key, float value);

  void set_param(int key, uint32_t value);

  float get_param_as_float(int key) const;

  uint32_t get_param_as_uint32(int key) const;

  bool is_have_param(int key) const;
};

// TODO(me): 实现电机意外断开连接的自动恢复机制
// TODO(me): 可以试试链表实现的无锁队列
template<typename T>
class Queue {
private:
    mutable std::mutex mutex_;
    std::queue<T> queue_;
    std::condition_variable cond_;
    bool shutdown_;

public:
    Queue();
    ~Queue() = default;
    
    Queue(const Queue&) = delete;
    Queue& operator=(const Queue&) = delete;
    
    bool try_push(T value);
    bool try_pop(T& value);
    bool pop(T& value);
    size_t size() const;
    bool empty() const;
    void clear();
    void shutdown();
    bool is_shutdown() const;
};

/**
 * @brief motor control class 电机控制类
 *
 * 使用USB转CAN进行通信，linux做虚拟串口
 */
class Motor_Control {
public:
  /*
   * @brief 定义电机控制对象
   * @param serial 串口对象
   * 默认串口为/dev/ttyACM0
   */
  Motor_Control(const std::string &serial_port, Serial::speed_t seial_baud);
    //  const std::unordered_map<Motor_id, DmActData>& dmact_data);

  ~Motor_Control();

  void get_motor_data();
  /*
   * @brief enable the motor 使能电机
   * @param motor 电机对象
   */
  void enable();

  void hscant_init(const std::string &_port);
  void hscant_deinit(const std::string &_port);
  // void write();
  // void read();
  /*
   * @brief refresh motor status 刷新电机状态
   * @param motor object 电机对象
   */
  void refresh_motor_status(const Motor &motor);
  void refresh_allmotor_status();

  void disable();
  void set_zero_position();

  void control_mit(Motor &DM_Motor, float kp, float kd, float q, float dq,
                   float tau);
  void control_pos_vel(Motor &DM_Motor, float pos, float vel);
  void control_vel(Motor &DM_Motor, float vel);

  /*
   * @description 函数库内部调用,用于解算电机can线返回的电机参数
   */
  void receive();

  void receive_param();

  

  /**
   * @brief add motor to class 添加电机
   * @param DM_Motor : motor object 电机对象
   */
  // void addMotor(Motor *DM_Motor);
  void addMotor(std::initializer_list<std::shared_ptr<Motor>> Motor_list);

  /*
   * @description: read motor register param
   * 读取电机内部寄存器参数，具体寄存器列表请参考达妙的手册
   * @param DM_Motor: motor object 电机对象
   * @param RID: register id 寄存器ID  example: damiao::UV_Value
   * @return: motor param 电机参数 如果没查询到返回的参数为0
   */
  float read_motor_param(Motor &DM_Motor, uint8_t RID);

  /*
   * @description: switch control mode 切换电机控制模式
   * @param DM_Motor: motor object 电机对象
   * @param mode: control mode 控制模式 like:damiao::MIT_MODE,
   * damiao::POS_VEL_MODE, damiao::VEL_MODE, damiao::POS_FORCE_MODE
   */
  bool switchControlMode(Motor &DM_Motor, Control_Mode mode);

  /*
   * @description: change motor param 修改电机内部寄存器参数
   * 具体寄存器列表请参考达妙手册
   * @param DM_Motor: motor object 电机对象
   * @param RID: register id 寄存器ID
   * @param data: param data
   * 参数数据,大部分数据是float类型，其中如果是uint32类型的数据也可以直接输入整型的就行，函数内部有处理
   * @return: bool true or false  是否修改成功
   */
  bool change_motor_param(Motor &DM_Motor, uint8_t RID, float data);

  /*
   * @description: save all param to motor flash 保存电机的所有参数到flash里面
   * @param DM_Motor: motor object 电机对象
   * 电机默认参数不会写到flash里面，需要进行写操作
   */
  void save_motor_param(Motor &DM_Motor);

  /*
   * @description: change motor limit param
   * 修改电机限制参数，这个修改的不是电机内部的寄存器参数，而是电机的限制参数
   * @param DM_Motor: motor object 电机对象
   * @param P_MAX: position max 位置最大值
   * @param Q_MAX: velocity max 速度最大值
   * @param T_MAX: torque max 扭矩最大值
   */
  static void changeMotorLimit(Motor &DM_Motor, float P_MAX, float Q_MAX,
                               float T_MAX);

private:
  void control_cmd(Motor_id id, uint8_t cmd);
  void write_motor_param(Motor &DM_Motor, uint8_t RID, const uint8_t data[4]);
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
  // std::unique_ptr<std::thread> rec_thread;
  std::thread update_thread;
  std::atomic<bool> stop_update_thread_;

  std::unordered_map<Motor_id, std::shared_ptr<Motor>> motors;
  std::unique_ptr<Serial::SerialPort> serial_;
  Queue<can_send_frame> send_queue;
  
  // ! 似乎是一个由用户维护的数据，用于集中管理单个串口的多个电机
  // ! 配套的api有MotorContorl构造函数、write、read函数
  // TODO(me): 暂时不用像这样暴露整个数据，后续修改
  std::unordered_map<Motor_id, DmActData> dmact_data;
  can_send_frame send_data;         // send data frame
  CAN_Receive_Frame receive_data{}; // receive data frame
};

}; // namespace damiao

#endif