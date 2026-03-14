#include "motor.h"

using namespace std::chrono_literals;
namespace motor {

Limit_param limit_param[Num_Of_Motor] = {
    {12.5, 30, 10, 500, 5},         // DM4310
    {12.5, 50, 10, 500, 5},         // DM4310_48V
    {12.5, 10, 28, 500, 5},         // DM4340
    {12.5, 10, 28, 500, 5},         // DM4340_48V
    {12.5, 45, 12, 500, 5},         // DM6006
    {12.5, 45, 20, 500, 5},         // DM8006
    {12.5, 45, 54, 500, 5},         // DM8009
    {12.5, 25, 200, 500, 5},        // DM10010L
    {12.5, 20, 200, 500, 5},        // DM10010
    {12.5, 280, 1, 500, 5},         // DMH3510
    {12.5, 45, 10, 500, 5},         // DMH6215
    {12.5, 45, 10, 500, 5},         // DMG6220
    {4 * M_PI, 33, 14, 500, 5},     // RS00 DQ50? T17?
    {4 * M_PI, 44, 17, 500, 5},     // RS01
    {4 * M_PI, 44, 17, 500, 5},     // RS02
    {4 * M_PI, 20, 60, 5000, 100},  // RS03 DQ50?
    {4 * M_PI, 15, 120, 5000, 100}, // RS04
    {4 * M_PI, 50, 5.5, 500, 5},    // RS05 DQ33? T17?
    {4 * M_PI, 50, 36, 5000, 100},  // RS06 DQ20? T60?
};

Motor::Motor(MotorType motor_type, Motor_id Slave_id, Motor_id Master_id)
    : Master_id(Master_id), Slave_id(Slave_id), motor_type(motor_type) {
  this->limit_param = motor::limit_param[motor_type];

  LOGI("Motor created: Slave ID %u, Master ID %u, Type %d", Slave_id, Master_id,
       motor_type);
  LOGI("limit_param: Q_MAX %.2f, TAU_MAX %.2f, KP_MAX %.2f, KD_MAX %.2f",
        this->limit_param.Q_MAX, this->limit_param.TAU_MAX,
        this->limit_param.KP_MAX, this->limit_param.KD_MAX);
}

void Motor::receive_data(float q, float dq, float tau) {
  this->state_q = q;
  this->state_dq = dq;
  this->state_tau = tau;
}


Motor_Control::Motor_Control(sockcanpp::CanDriver *hscant_handler)
    : hscant_handler(hscant_handler) {

  // const std::unordered_map<Motor_id, DmActData>& act_data)
  //   : act_data(act_data) {

  // for (const auto & item : act_data) {
  //   Motor *motor =
  //       new Motor(item.second.motorType, item.second.can_id,
  //                 item.second.mst_id); // 假设Motor的构造函数不需要参数
  //   addMotor(motor);
  // }
  
  stop_update_thread_ = false;
  update_thread = std::thread(&Motor_Control::update_motor, this);
  recv_thread = std::thread([this]() {
    while (!stop_update_thread_) {
      get_motor_data();
    }
  });
}

Motor_Control::~Motor_Control() {
  LOGW("Motor_Control Exited");
  for (const auto &pair : motors) {
    Motor_id id = pair.first;
    control_mit(*motors[id], 0, 0, 0, 0, 0);
  }

  stop_update_thread_ = true;
  if (update_thread.joinable()) {
    update_thread.join();
  }
  send_queue.shutdown();
}

void Motor_Control::enable() {
  for (auto &it : motors) {
    for (int i = 0; i < 5; i++) {
      control_cmd(it.second->GetSlaveId(), 0xFC);
      usleep(200);
    }
    usleep(200);
  }
}

void Motor_Control::refresh_motor_status(const Motor &motor) {


  Can_Send_Frame send_data;
  uint32_t id = 0x7FF;
  uint8_t can_low = motor.GetSlaveId() & 0xff;         // id low 8 bit
  uint8_t can_high = (motor.GetSlaveId() >> 8) & 0xff; // id high 8 bit
  std::array<uint8_t, 8> data_buf = {can_low, can_high, 0xCC, 0x00,
                                     0x00,    0x00,     0x00, 0x00};
  send_data.modify(id, data_buf.data());
  // serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
  send_queue.try_push(send_data);
}
// TODO(me): 数据包长度不对，发送数据包需要修改
void Motor_Control::refresh_allmotor_status() {
  Can_Send_Frame send_data;
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
  }
}

void Motor_Control::disable() {
  for (auto &it : motors) {
    for (int i = 0; i < 10; i++) {
      control_cmd(it.second->GetSlaveId(), 0xFD);
      usleep(150);
    }
  }
}

void Motor_Control::set_zero_position() {
  for (auto &it : motors) {
    control_cmd(it.second->GetSlaveId(), 0xFE);
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
  Can_Send_Frame send_data;
  Motor_id id = motor.GetSlaveId();
  if (motors.find(id) == motors.end()) {
    LOGE("[motor] Cant find motor with given id");
    return;
    // throw std::runtime_error("Motor_Control id not found");
  }
  auto &m = motors[id];
  Limit_param limit_param_cmd = m->get_limit_param();
  uint16_t kp_uint = float_to_uint(kp, 0, limit_param_cmd.KP_MAX, 12);
  uint16_t kd_uint = float_to_uint(kd, 0, limit_param_cmd.KD_MAX, 12);
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
}

void Motor_Control::control_pos_vel(Motor &DM_Motor, float pos, float vel) {
  Can_Send_Frame send_data;
  Motor_id id = DM_Motor.GetSlaveId();
  if (motors.find(id) == motors.end()) {
    LOGE("[motor] Cant find motor with given id");
    return;
  }
  std::array<uint8_t, 8> data_buf{};
  memcpy(data_buf.data(), &pos, sizeof(float));
  memcpy(data_buf.data() + 4, &vel, sizeof(float));
  id += POS_MODE;
  send_data.modify(id, data_buf.data());
  send_queue.try_push(send_data);
}

void Motor_Control::control_vel(Motor &DM_Motor, float vel) {
  Can_Send_Frame send_data;
  Motor_id id = DM_Motor.GetSlaveId();
  if (motors.find(id) == motors.end()) {
    LOGE("[motor] Cant find motor with given id");
    return;
  }
  std::array<uint8_t, 8> data_buf = {0};
  memcpy(data_buf.data(), &vel, sizeof(float));
  id = id + SPEED_MODE;
  send_data.modify(id, data_buf.data());
  // serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
  send_queue.try_push(send_data);
}

void Motor_Control::addMotor(
    std::initializer_list<std::shared_ptr<Motor>> Motor_list) {
  for (const auto &motor : Motor_list) {
    motors.insert({motor->GetSlaveId(), motor});
    motors.insert({motor->GetMasterId(), motor});
  }
  disable();
}

void Motor_Control::addMotor(
    std::vector<std::shared_ptr<Motor>> Motor_list) {
  for (const auto &motor : Motor_list) {
    motors.insert({motor->GetSlaveId(), motor});
    motors.insert({motor->GetMasterId(), motor});
  }
  disable();
}

void Motor_Control::changeMotorLimit(Motor &DM_Motor, float P_MAX, float Q_MAX,
                                     float T_MAX) {
  limit_param[DM_Motor.GetMotorType()] = {P_MAX, Q_MAX, T_MAX};
}

void Motor_Control::control_cmd(Motor_id id, uint8_t cmd) {
  std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff,
                                     0xff, 0xff, 0xff, cmd};
  // Can_Send_Frame send_data;
  // send_data.modify(id, data_buf.data());
  // serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
  // send_queue.try_push(send_data);
  // hscant_handler->send_frame(send_data.canId, send_data.data, 8);
  struct can_frame can_frame {};
  can_frame.can_id = id;
  can_frame.can_dlc = 8;
  std::copy(data_buf.begin(), data_buf.end(), can_frame.data);
  hscant_handler->sendMessage(sockcanpp::CanMessage(can_frame));

}

// 所有发送数据集中在这里，不阻塞主线程
void Motor_Control::update_motor() {
  while (!stop_update_thread_ || !send_queue.empty()) {
    Can_Send_Frame frame;
    if (send_queue.pop(frame)) {
      // define in linux/can.h
      struct can_frame can_frame {};
      can_frame.can_id = frame.canId;
      can_frame.can_dlc = 8;
      std::copy(frame.data, frame.data + 8, can_frame.data);
      auto msg = sockcanpp::CanMessage(can_frame);
      hscant_handler->sendMessage(msg);
    }
  }
}

void Motor_Control::write() {
  for (const auto &m : act_data) {
    int motor_id = m.first; // can_id
    if (motors.find(motor_id) == motors.end()) {
    }
    auto &it = motors[motor_id];

    control_mit(*it, m.second.kp, m.second.kd, m.second.cmd_pos,
                m.second.cmd_vel, m.second.cmd_effort);
  }
}

void Motor_Control::read() {
  for (auto &m : act_data) {
    int motor_id = m.first; // can_id
    if (motors.find(motor_id) == motors.end()) {
    }
    auto &it = motors[motor_id];

    m.second.pos = it->Get_Position();
    m.second.vel = it->Get_Velocity();
    m.second.effort = it->Get_tau();
  }
}

void Motor_Control::get_motor_data() {
  // if (!hscant_handler->recv_frame(receive_data_t.canId, receive_data_t.canData, 8)) {
    // return;
  // }
  if (!hscant_handler->waitForMessages(1ms)) {
    return;
  }

  Can_Receive_Frame receive_data_t;
  const auto msg = hscant_handler->readMessage();
  receive_data_t.canId = msg.getCanId();
  auto frame_data = msg.getFrameData();
  std::copy(frame_data.begin(), frame_data.end(), receive_data_t.canData);
  // TODO(me): 这里需要处理错误信息
  const auto hasBusError = msg.hasBusError();
  const auto hasBusOffError = msg.hasBusOffError();
  const auto hasControllerProblem = msg.hasControllerProblem();
  const auto hasControllerRestarted = msg.hasControllerRestarted();
  const auto hasErrorCounter = msg.hasErrorCounter();
  const auto hasLostArbitration = msg.hasLostArbitration();
  const auto hasProtocolViolation = msg.hasProtocolViolation();
  const auto hasTransceiverStatus = msg.hasTransceiverStatus();
  const auto missingAckOnTransmit = msg.missingAckOnTransmit();
  const auto isTxTimeout = msg.isTxTimeout();
  const auto controllerError = msg.getControllerError();
  const auto protocolError = msg.getProtocolError();
  const auto transceiverError = msg.getTransceiverError();
  const auto txErrorCounter = msg.getTxErrorCounter();
  const auto rxErrorCounter = msg.getRxErrorCounter();
  const auto arbitrationLostInBit = msg.arbitrationLostInBit();
  // print detailed error info
  if (hasBusError) {
    LOGW("  Bus Error");
    return;
  }
  if (hasBusOffError) {
    LOGW("  Bus Off Error");
    return;
  } 
  if (hasControllerProblem) {
    LOGW("  Controller Problem");
    return;
  }
  if (hasControllerRestarted) {
    LOGW("  Controller Restarted");
    return;
  }
  if (hasErrorCounter) {
    LOGW("  Error Counter - TX: %lu, RX: %lu", txErrorCounter, rxErrorCounter);
    return;
  }
  if (hasLostArbitration) {
    LOGW("  Lost Arbitration at Bit: %u", arbitrationLostInBit);
    return;
  }
  if (hasProtocolViolation) { 
    LOGW("  Protocol Violation");
    return;
    // check protocolError for more details
  }
  if (hasTransceiverStatus) {
    LOGW("  Transceiver Status Error");
    return;
    // check transceiverError for more details
  }
  if (missingAckOnTransmit) {
    LOGW("  Missing Acknowledgment on Transmit");
    return;
  }
  if (isTxTimeout) {
    LOGW("  Transmission Timeout");
    return;
  }


  static auto uint_to_float = [](uint16_t x, float xmin, float xmax,
                                  uint8_t bits) -> float {
    float span = xmax - xmin;
    float data_norm = float(x) / ((1 << bits) - 1);
    float data = data_norm * span + xmin;
    return data;
  };

  auto &data = receive_data_t.canData;

  uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
  uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
  uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];

  if (motors.find(receive_data_t.canId) == motors.end()) {
    LOGW("[motor] Cant find motor by id: %X", receive_data_t.canId);
    return;
  }
  auto m = motors[receive_data_t.canId];
  Limit_param limit_param_receive = m->get_limit_param();
  float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX,
                                  limit_param_receive.Q_MAX, 16);
  float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX,
                                    limit_param_receive.DQ_MAX, 12);
  float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX,
                                    limit_param_receive.TAU_MAX, 12);

  // std::cout << receive_data_t.canId << " " << receive_q << std::endl;
  m->receive_data(receive_q, receive_dq, receive_tau);
}

} // namespace motor
