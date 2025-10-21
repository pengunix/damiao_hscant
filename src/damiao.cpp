#include "damiao.h"
#include <iostream>
namespace damiao {

Limit_param limit_param[Num_Of_Motor] = {
    {12.5, 30, 10},  // DM4310
    {12.5, 50, 10},  // DM4310_48V
    {12.5, 10, 28},  // DM4340
    {12.5, 10, 28},  // DM4340_48V
    {12.5, 45, 12},  // DM6006
    {12.5, 45, 20},  // DM8006
    {12.5, 45, 54},  // DM8009
    {12.5, 25, 200}, // DM10010L
    {12.5, 20, 200}, // DM10010
    {12.5, 280, 1},  // DMH3510
    {12.5, 45, 10},  // DMH6215
    {12.5, 45, 10}   // DMG6220
};

Motor::Motor(DM_Motor_Type Motor_Type, Motor_id Slave_id, Motor_id Master_id)
    : Master_id(Master_id), Slave_id(Slave_id), Motor_Type(Motor_Type) {
  this->limit_param = damiao::limit_param[Motor_Type];
}

void Motor::receive_data(float q, float dq, float tau) {
  this->state_q = q;
  this->state_dq = dq;
  this->state_tau = tau;
}

void Motor::set_param(int key, float value) {
  ValueType v{};
  v.value.floatValue = value;
  v.isFloat = true;
  param_map[key] = v;
}

void Motor::set_param(int key, uint32_t value) {
  ValueType v{};
  v.value.uint32Value = value;
  v.isFloat = false;
  param_map[key] = v;
}

float Motor::get_param_as_float(int key) const {
  auto it = param_map.find(key);
  if (it != param_map.end()) {
    if (it->second.isFloat) {
      return it->second.value.floatValue;
    } else {
      return 0;
    }
  }
  return 0;
}

uint32_t Motor::get_param_as_uint32(int key) const {
  auto it = param_map.find(key);
  if (it != param_map.end()) {
    if (!it->second.isFloat) {
      return it->second.value.uint32Value;
    } else {
      return 0;
    }
  }
  return 0;
}

bool Motor::is_have_param(int key) const {
  return param_map.find(key) != param_map.end();
}

Motor_Control::Motor_Control(const std::string &serial_port,
                             Serial::speed_t serial_baud) {

  // const std::unordered_map<Motor_id, DmActData>& dmact_data)
  //   : dmact_data(dmact_data) {

  // for (const auto & item : dmact_data) {
  //   Motor *motor =
  //       new Motor(item.second.motorType, item.second.can_id,
  //                 item.second.mst_id); // 假设Motor的构造函数不需要参数
  //   addMotor(motor);
  // }

  serial_ = std::make_unique<Serial::SerialPort>(serial_port, serial_baud);
  update_thread =
      std::thread(&Motor_Control::update_motor, this);
  stop_update_thread_ = false;

  hscant_init(serial_port);
}

// TODO(me): 模块完成后检查类中是否有动态存储期成员需要释放
Motor_Control::~Motor_Control() {
  for (const auto &pair : motors) {
    Motor_id id = pair.first;
    control_mit(*motors[id], 0, 0.3, 0, 0, 0);
  }

  stop_update_thread_ = true;
  send_queue.shutdown();
  if (update_thread.joinable()) {
    update_thread.join();
  }

  if (serial_->is_Open()) {
    serial_->close();
  }
}

// TODO(me): 这个很不妙啊，需要改进
std::string getPortString(const std::string &_port) {
  static const std::unordered_map<std::string, std::string> portMap = {
      {"/dev/ttyACM0", "r_can0"},
      {"/dev/ttyACM1", "r_can1"},
      {"/dev/ttyACM2", "r_can2"},
      {"/dev/ttyACM3", "r_can3"}};

  auto it = portMap.find(_port);
  if (it != portMap.end()) {
    return it->second;
  }
  // TODO(me): 增加logger，提示没有找到对应的端口，使用默认端口
  return "r_can0";
}

void Motor_Control::hscant_init(const std::string &_port) {
  // 初始化阶段不用同步队列发送
  std::string cmd = getPortString(_port);
  serial_->send((uint8_t *)cmd.data(), cmd.size());
  usleep(200);
  // S8\r 选择波特率为1000000
  uint8_t data_buf0[3] = {'S', '8', '\r'};
  serial_->send((uint8_t *)&data_buf0, sizeof(data_buf0));
  usleep(200);

  // U1\r
  uint8_t data_buf1[3] = {'U', '1', '\r'};
  serial_->send((uint8_t *)&data_buf1, sizeof(data_buf1));
  usleep(200);

  // O\r
  uint8_t data_buf2[2] = {'O', '\r'};
  serial_->send((uint8_t *)&data_buf2, sizeof(data_buf2));
  usleep(200);
}

// TODO(me): 实现四路can口的deinit
void Motor_Control::hscant_deinit(const std::string &_port) {}

void Motor_Control::enable() {
  for (auto &it : motors) {
    control_cmd(it.second->GetSlaveId(), 0xFC);
  }
}

// 读电机反馈命令
// TODO(me): 该api似乎没有效果，有控制指令用不到它，没控制指令又不会刷新数据
void Motor_Control::refresh_motor_status(const Motor &motor) {
  uint32_t id = 0x7FF;
  uint8_t can_low = motor.GetSlaveId() & 0xff;         // id low 8 bit
  uint8_t can_high = (motor.GetSlaveId() >> 8) & 0xff; // id high 8 bit
  std::array<uint8_t, 8> data_buf = {can_low, can_high, 0xCC, 0x00,
                                     0x00,    0x00,     0x00, 0x00};
  send_data.modify(id, data_buf.data());
  // serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
  send_queue.try_push(send_data);
}
// TODO(me): 为啥还不行
void Motor_Control::refresh_allmotor_status() {
  uint32_t id = 0x7FF;
  for (auto &it : motors) {
    const Motor &motor = *(it.second);
    uint8_t can_low = motor.GetSlaveId() & 0xff;         // id low 8 bit
    uint8_t can_high = (motor.GetSlaveId() >> 8) & 0xff; // id high 8 bit
    std::array<uint8_t, 8> data_buf = {can_low, can_high, 0xCC, 0x00,
                                       0x00,    0x00,     0x00, 0x00};
    send_data.modify(id, data_buf.data());
    // serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
    send_queue.try_push(send_data);
    usleep(200);
  }
}

void Motor_Control::disable() {
  for (auto &it : motors) {
    control_cmd(it.second->GetSlaveId(), 0xFD);
  }
}

void Motor_Control::set_zero_position() {
  for (auto &it : motors) {
    control_cmd(it.second->GetSlaveId(), 0xFE);
    usleep(2000);
  }
}

void Motor_Control::control_mit(Motor &motor, float kp, float kd, float q,
                                float dq, float tau) {
  static auto float_to_uint = [](float x, float xmin, float xmax,
                                 uint8_t bits) -> uint16_t {
    float span = xmax - xmin;
    float data_norm = (x - xmin) / span;
    uint16_t data_uint = data_norm * ((1 << bits) - 1);
    return data_uint;
  };
  Motor_id id = motor.GetSlaveId();
  if (motors.find(id) == motors.end()) {
    // TODO(me): 完全去掉异常，提示并终止程序即可
    // FATAL error
    // throw std::runtime_error("Motor_Control id not found");
  }
  auto &m = motors[id];
  uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
  uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
  Limit_param limit_param_cmd = m->get_limit_param();
  uint16_t q_uint =
      float_to_uint(q, -limit_param_cmd.Q_MAX, limit_param_cmd.Q_MAX, 16);
  uint16_t dq_uint =
      float_to_uint(dq, -limit_param_cmd.DQ_MAX, limit_param_cmd.DQ_MAX, 12);
  uint16_t tau_uint =
      float_to_uint(tau, -limit_param_cmd.TAU_MAX, limit_param_cmd.TAU_MAX, 12);

  std::array<uint8_t, 8> data_buf{};
  data_buf[0] = (q_uint >> 8) & 0xff;
  data_buf[1] = q_uint & 0xff;
  data_buf[2] = dq_uint >> 4;
  data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
  data_buf[4] = kp_uint & 0xff;
  data_buf[5] = kd_uint >> 4;
  data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
  data_buf[7] = tau_uint & 0xff;

  send_data.modify(id, data_buf.data());
  // serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
  send_queue.try_push(send_data);
  usleep(200);
}

void Motor_Control::control_pos_vel(Motor &DM_Motor, float pos, float vel) {
  Motor_id id = DM_Motor.GetSlaveId();
  if (motors.find(id) == motors.end()) {
    // TODO(me): 完全去掉异常，提示并终止程序即可
    // throw std::runtime_error("POS_VEL ERROR : Motor_Control id not found");
    // FATAL error
  }
  std::array<uint8_t, 8> data_buf{};
  memcpy(data_buf.data(), &pos, sizeof(float));
  memcpy(data_buf.data() + 4, &vel, sizeof(float));
  id += POS_MODE;
  send_data.modify(id, data_buf.data());
  // serial_->send(reinterpret_cast<uint8_t *>(&send_data),
                // sizeof(can_send_frame));
  send_queue.try_push(send_data);
}

void Motor_Control::control_vel(Motor &DM_Motor, float vel) {
  Motor_id id = DM_Motor.GetSlaveId();
  if (motors.find(id) == motors.end()) {
    // TODO(me): 完全去掉异常，提示并终止程序即可
    // throw std::runtime_error("VEL ERROR : id not found");
  }
  std::array<uint8_t, 8> data_buf = {0};
  memcpy(data_buf.data(), &vel, sizeof(float));
  id = id + SPEED_MODE;
  send_data.modify(id, data_buf.data());
  // serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
  send_queue.try_push(send_data);
}

// TODO(me): 数据包修正
void Motor_Control::receive_param() {
  // 待修正，临时终止函数运行
  return;

  if (receive_data.FrameHeader == 0x11 &&
      receive_data.frameEnd == 0x55) // receive success
  {
    auto &data = receive_data.canData;
    if (data[2] == 0x33 or data[2] == 0x55) {
      uint32_t slaveID = (uint32_t(data[1]) << 8) | data[0];
      uint8_t RID = data[3];
      if (motors.find(slaveID) == motors.end()) {
        // can not found motor id
        return;
      }
      if (is_in_ranges(RID)) {
        uint32_t data_uint32 = (uint32_t(data[7]) << 24) |
                               (uint32_t(data[6]) << 16) |
                               (uint32_t(data[5]) << 8) | data[4];
        motors[slaveID]->set_param(RID, data_uint32);
      } else {
        float data_float = uint8_to_float(data + 4);
        motors[slaveID]->set_param(RID, data_float);
      }
    }
    return;
  }
}

/**
 * @brief add motor to class 添加电机
 * @param DM_Motor : motor object 电机对象的地址
 */
// void Motor_Control::addMotor(Motor *DM_Motor) {
//   motors.insert({DM_Motor->GetSlaveId(), DM_Motor});
//   motors.insert({DM_Motor->GetMasterId(), DM_Motor});
// }

void Motor_Control::addMotor(
    std::initializer_list<std::shared_ptr<Motor>> Motor_list) {
  for (const auto &motor_ptr : Motor_list) {
    // TODO(me); 检查电机初始化状态
    motors.insert({motor_ptr->GetSlaveId(), motor_ptr});
    motors.insert({motor_ptr->GetMasterId(), motor_ptr});
  }
  disable();
}

/*
 * @description: read motor register param
 * 读取电机内部寄存器参数，具体寄存器列表请参考达妙的手册
 * @param DM_Motor: motor object 电机对象
 * @param RID: register id 寄存器ID  example: damiao::UV_Value
 * @return: motor param 电机参数 如果没查询到返回的参数为0
 */
float Motor_Control::read_motor_param(Motor &DM_Motor, uint8_t RID) {
  uint32_t id = DM_Motor.GetSlaveId();
  uint8_t can_low = id & 0xff;
  uint8_t can_high = (id >> 8) & 0xff;
  std::array<uint8_t, 8> data_buf{can_low, can_high, 0x33, RID,
                                  0x00,    0x00,     0x00, 0x00};
  send_data.modify(0x7FF, data_buf.data());
  // serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
  send_queue.try_push(send_data);
  for (uint8_t i = 0; i < max_retries; i++) {
    usleep(retry_interval);
    // receive_param();
    if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID)) {
      if (is_in_ranges(RID)) {
        return float(motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID));
      } else {
        return motors[DM_Motor.GetSlaveId()]->get_param_as_float(RID);
      }
    }
  }

  return 0;
}

/*
 * @description: switch control mode 切换电机控制模式
 * @param DM_Motor: motor object 电机对象
 * @param mode: control mode 控制模式 like:damiao::MIT_MODE,
 * damiao::POS_VEL_MODE, damiao::VEL_MODE, damiao::POS_FORCE_MODE
 */
bool Motor_Control::switchControlMode(Motor &DM_Motor, Control_Mode mode) {
  uint8_t write_data[4] = {(uint8_t)mode, 0x00, 0x00, 0x00};
  uint8_t RID = 10;
  write_motor_param(DM_Motor, RID, write_data);
  if (motors.find(DM_Motor.GetSlaveId()) == motors.end()) {
    return false;
  }
  for (uint8_t i = 0; i < max_retries; i++) {
    usleep(retry_interval);
    // receive_param();
    if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID)) {
      return motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID) == mode;
    }
  }
  return false;
}

/*
 * @description: change motor param 修改电机内部寄存器参数
 * 具体寄存器列表请参考达妙手册
 * @param DM_Motor: motor object 电机对象
 * @param RID: register id 寄存器ID
 * @param data: param data
 * 参数数据,大部分数据是float类型，其中如果是uint32类型的数据也可以直接输入整型的就行，函数内部有处理
 * @return: bool true or false  是否修改成功
 */
bool Motor_Control::change_motor_param(Motor &DM_Motor, uint8_t RID,
                                       float data) {
  if (is_in_ranges(RID)) {
    uint32_t data_uint32 = float_to_uint32(data);
    uint8_t *data_uint8;
    data_uint8 = (uint8_t *)&data_uint32;
    write_motor_param(DM_Motor, RID, data_uint8);
  } else {
    // is float
    uint8_t *data_uint8;
    data_uint8 = (uint8_t *)&data;
    write_motor_param(DM_Motor, RID, data_uint8);
  }
  if (motors.find(DM_Motor.GetSlaveId()) == motors.end()) {
    return false;
  }
  for (uint8_t i = 0; i < max_retries; i++) {
    usleep(retry_interval);
    // receive_param();
    if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID)) {
      if (is_in_ranges(RID)) {
        return motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID) ==
               float_to_uint32(data);
      } else {
        return fabsf(motors[DM_Motor.GetSlaveId()]->get_param_as_float(RID) -
                     data) < 0.1f;
      }
    }
  }
  return false;
}

/*
 * @description: save all param to motor flash 保存电机的所有参数到flash里面
 * @param DM_Motor: motor object 电机对象
 * 电机默认参数不会写到flash里面，需要进行写操作
 */
void Motor_Control::save_motor_param(Motor &DM_Motor) {
  disable();
  uint32_t id = DM_Motor.GetSlaveId();
  uint8_t id_low = id & 0xff;
  uint8_t id_high = (id >> 8) & 0xff;
  std::array<uint8_t, 8> data_buf{id_low, id_high, 0xAA, 0x01,
                                  0x00,   0x00,    0x00, 0x00};
  send_data.modify(0x7FF, data_buf.data());
  // serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
  send_queue.try_push(send_data);
  usleep(100000); // 100ms wait for save
}

/*
 * @description: change motor limit param
 * 修改电机限制参数，这个修改的不是电机内部的寄存器参数，而是电机的限制参数
 * @param DM_Motor: motor object 电机对象
 * @param P_MAX: position max 位置最大值
 * @param Q_MAX: velocity max 速度最大值
 * @param T_MAX: torque max 扭矩最大值
 */
void Motor_Control::changeMotorLimit(Motor &DM_Motor, float P_MAX, float Q_MAX,
                                     float T_MAX) {
  limit_param[DM_Motor.GetMotorType()] = {P_MAX, Q_MAX, T_MAX};
}

void Motor_Control::control_cmd(Motor_id id, uint8_t cmd) {
  std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff,
                                     0xff, 0xff, 0xff, cmd};
  send_data.modify(id, data_buf.data());
  // serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
  send_queue.try_push(send_data);
  usleep(200);
}

void Motor_Control::write_motor_param(Motor &DM_Motor, uint8_t RID,
                                      const uint8_t data[4]) {
  uint32_t id = DM_Motor.GetSlaveId();
  uint8_t can_low = id & 0xff;
  uint8_t can_high = (id >> 8) & 0xff;
  std::array<uint8_t, 8> data_buf{can_low, can_high, 0x55, RID,
                                  0x00,    0x00,     0x00, 0x00};
  data_buf[4] = data[0];
  data_buf[5] = data[1];
  data_buf[6] = data[2];
  data_buf[7] = data[3];
  send_data.modify(0x7FF, data_buf.data());

  // serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
  send_queue.try_push(send_data);
}

// 所有发送数据集中在这里，不阻塞主线程
void Motor_Control::update_motor() {
  while (!stop_update_thread_) {
    can_send_frame frame;
    if (send_queue.pop(frame)) {
      serial_->send((uint8_t *)&frame, sizeof(can_send_frame));
      get_motor_data();
    }
  }
}

// void Motor_Control::write() {
//   for (const auto &m : dmact_data) {
//     int motor_id = m.first; //这里指的是can_id
//     if (motors.find(motor_id) == motors.end()) {
//     }
//     auto &it = motors[motor_id];

//     control_mit(*it, m.second.kp, m.second.kd, m.second.cmd_pos,
//                 m.second.cmd_vel,
//                 m.second.cmd_effort);
//   }
// }

// void Motor_Control::read() {
//   for (auto &m : dmact_data) {
//     int motor_id = m.first; //这里指的是can_id
//     if (motors.find(motor_id) == motors.end()) {
//     }
//     auto &it = motors[motor_id];

//     m.second.pos = it->Get_Position();
//     m.second.vel = it->Get_Velocity();
//     m.second.effort = it->Get_tau();

//   }
// }

// TODO(me): 修改hscant固件，支持透传模式，删除ascii转换逻辑
uint8_t hexCharToNibble(uint8_t c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return 10 + (c - 'A');
  if (c >= 'a' && c <= 'f')
    return 10 + (c - 'a'); // 支持小写
  return 0;                // 处理无效字符
}

void Motor_Control::get_motor_data() {
  serial_->recv((uint8_t *)&receive_data, sizeof(CAN_Receive_Frame));

  if (receive_data.FrameHeader == 0x74 && receive_data.frameEnd == 0x0D) {
    static auto uint_to_float = [](uint16_t x, float xmin, float xmax,
                                   uint8_t bits) -> float {
      float span = xmax - xmin;
      float data_norm = float(x) / ((1 << bits) - 1);
      float data = data_norm * span + xmin;
      return data;
    };

    uint8_t data[8];
    for (int i = 0; i < 8; i++) {
      uint8_t highChar = receive_data.canData[2 * i];
      uint8_t lowChar = receive_data.canData[2 * i + 1];

      uint8_t highNibble = hexCharToNibble(highChar);
      uint8_t lowNibble = hexCharToNibble(lowChar);

      data[i] = (highNibble << 4) | lowNibble;
    }

    uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
    uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
    uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];

    uint32_t canId = (hexCharToNibble(receive_data.canId[0]) << 8) |
                     (hexCharToNibble(receive_data.canId[1]) << 4) |
                     hexCharToNibble(receive_data.canId[2]);

    if (motors.find(canId) == motors.end()) {
      return;
    }
    auto m = motors[canId];
    Limit_param limit_param_receive = m->get_limit_param();
    float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX,
                                    limit_param_receive.Q_MAX, 16);
    float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX,
                                     limit_param_receive.DQ_MAX, 12);
    float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX,
                                      limit_param_receive.TAU_MAX, 12);

    m->receive_data(receive_q, receive_dq, receive_tau);
  } else {
    // TODO(me): 提示错误接收帧
  }
}



template<typename T>
Queue<T>::Queue() : shutdown_(false) {
}

// 非阻塞推送
template<typename T>
bool Queue<T>::try_push(T value) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (shutdown_) {
        return false;
    }
    
    queue_.push(std::move(value));
    cond_.notify_one();
    return true;
}

// 非阻塞获取
template<typename T>
bool Queue<T>::try_pop(T& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (queue_.empty() || shutdown_) {
        return false;
    }
    
    value = std::move(queue_.front());
    queue_.pop();
    return true;
}

// 阻塞获取
template<typename T>
bool Queue<T>::pop(T& value) {
    std::unique_lock<std::mutex> lock(mutex_);
    
    cond_.wait(lock, [this]() { 
        return !queue_.empty() || shutdown_; 
    });
    
    if (shutdown_ && queue_.empty()) {
        return false;
    }
    
    value = std::move(queue_.front());
    queue_.pop();
    return true;
}

// 队列大小
template<typename T>
size_t Queue<T>::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
}

// 队列是否为空
template<typename T>
bool Queue<T>::empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
}

// 清空队列
template<typename T>
void Queue<T>::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    while (!queue_.empty()) {
        queue_.pop();
    }
}

// 关闭队列
template<typename T>
void Queue<T>::shutdown() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
    }
    cond_.notify_all();
}

// 检查是否已关闭
template<typename T>
bool Queue<T>::is_shutdown() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shutdown_;
}

} // namespace damiao
