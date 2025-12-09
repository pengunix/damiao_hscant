#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "HSCanT.h"
#include "damiao.h"
#include "rclcpp/rclcpp.hpp"

#include "damiao_hscant/msg/dm_command.hpp"
#include "damiao_hscant/msg/dm_state.hpp"

using namespace std::chrono_literals;

// Constants
constexpr damiao::DM_Motor_Type MotorType = damiao::DM8009;
constexpr float ZERO_POS = 1.5707963267948;
constexpr std::array<int, 3> FL_dir = {1, 1, -1};
constexpr std::array<int, 3> FR_dir = {1, -1, 1};
constexpr std::array<int, 3> BL_dir = {-1, 1, -1};
constexpr std::array<int, 3> BR_dir = {-1, -1, 1};
constexpr std::array<std::array<int, 3>, 4> Motor_dirs = {FL_dir, FR_dir, BL_dir, BR_dir};

struct dm_cmd {
  float q;
  float dq;
  float tau;
  float kp;
  float kd;
};

class DamiaoMotorNode : public rclcpp::Node {
public:
  DamiaoMotorNode() : Node("dm_main") {
    // 1. Initialize HSCanT
    hscant_ = std::make_unique<HSCanT::HSCanT>();
    if (!hscant_->is_valid()) {
      RCLCPP_ERROR(this->get_logger(), "HSCanT initialization failed!");
      rclcpp::shutdown();
      return;
    }

    // 2. Export Handlers
    auto FL_handler_ptr = hscant_->export_handler(0);
    auto FR_handler_ptr = hscant_->export_handler(1);
    auto BL_handler_ptr = hscant_->export_handler(2);
    auto BR_handler_ptr = hscant_->export_handler(3);

    // 3. Initialize Motors and Controllers
    // Front Left
    FL_M_ = {std::make_shared<damiao::Motor>(MotorType, 0x01, 0x11),
             std::make_shared<damiao::Motor>(MotorType, 0x02, 0x12),
             std::make_shared<damiao::Motor>(MotorType, 0x03, 0x13)};
    dm_FL_ = std::make_unique<damiao::Motor_Control>(FL_handler_ptr.get());
    dm_FL_->addMotor({FL_M_[0], FL_M_[1], FL_M_[2]});

    // Front Right
    FR_M_ = {std::make_shared<damiao::Motor>(MotorType, 0x01, 0x11),
             std::make_shared<damiao::Motor>(MotorType, 0x02, 0x12),
             std::make_shared<damiao::Motor>(MotorType, 0x03, 0x13)};
    dm_FR_ = std::make_unique<damiao::Motor_Control>(FR_handler_ptr.get());
    dm_FR_->addMotor({FR_M_[0], FR_M_[1], FR_M_[2]});

    // Back Left
    BL_M_ = {std::make_shared<damiao::Motor>(MotorType, 0x01, 0x11),
             std::make_shared<damiao::Motor>(MotorType, 0x02, 0x12),
             std::make_shared<damiao::Motor>(MotorType, 0x03, 0x13)};
    dm_BL_ = std::make_unique<damiao::Motor_Control>(BL_handler_ptr.get());
    dm_BL_->addMotor({BL_M_[0], BL_M_[1], BL_M_[2]});

    // Back Right
    BR_M_ = {std::make_shared<damiao::Motor>(MotorType, 0x01, 0x11),
             std::make_shared<damiao::Motor>(MotorType, 0x02, 0x12),
             std::make_shared<damiao::Motor>(MotorType, 0x03, 0x13)};
    dm_BR_ = std::make_unique<damiao::Motor_Control>(BR_handler_ptr.get());
    dm_BR_->addMotor({BR_M_[0], BR_M_[1], BR_M_[2]});

    // Move ownership of handlers to member variables to keep them alive
    FL_handler_ = std::move(FL_handler_ptr);
    FR_handler_ = std::move(FR_handler_ptr);
    BL_handler_ = std::move(BL_handler_ptr);
    BR_handler_ = std::move(BR_handler_ptr);

    // 4. Enable Sequence
    std::this_thread::sleep_for(1s);
    dm_FL_->enable();
    dm_FR_->enable();
    dm_BL_->enable();
    dm_BR_->enable();

    // 5. Check Connections
    RCLCPP_INFO(this->get_logger(), "=================================================");
    std::vector<float> init_pos = {
        FL_M_[0]->Get_Position(), FL_M_[1]->Get_Position(), FL_M_[2]->Get_Position(),
        FR_M_[0]->Get_Position(), FR_M_[1]->Get_Position(), FR_M_[2]->Get_Position(),
        BL_M_[0]->Get_Position(), BL_M_[1]->Get_Position(), BL_M_[2]->Get_Position(),
        BR_M_[0]->Get_Position(), BR_M_[1]->Get_Position(), BR_M_[2]->Get_Position()};

    for (size_t i = 0; i < init_pos.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "Motor %ld Initial Position: %.4f", i, init_pos[i]);
      // Safety check: exact 0.0 usually means no communication
      if (std::fabs(init_pos[0]) == 0.0000) {
        RCLCPP_ERROR(this->get_logger(), "Motor %ld may not be connected properly!", i);
        this->disable_all();
        rclcpp::shutdown();
        return;
      }
    }
    RCLCPP_INFO(this->get_logger(), "Damiao Motor Node Initialized.");
    RCLCPP_INFO(this->get_logger(), "=================================================");

    // 6. Initialize Commands
    init_commands();

    // 7. Initialize ROS Communication
    motor_state_msg_.joint_names = {
        "FL_M0", "FL_M1", "FL_M2", "FR_M0", "FR_M1", "FR_M2",
        "BL_M0", "BL_M1", "BL_M2", "BR_M0", "BR_M1", "BR_M2"};

    // QoS for high frequency control
    rclcpp::QoS qos_profile(128);
    qos_profile.reliable();

    sub_ = this->create_subscription<damiao_hscant::msg::DmCommand>(
        "/dm_cmd", qos_profile,
        std::bind(&DamiaoMotorNode::command_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<damiao_hscant::msg::DmState>("/dm_states", qos_profile);

    // 8. Start Control Loop Timer (100Hz = 10ms)
    timer_ = this->create_wall_timer(
        10ms, std::bind(&DamiaoMotorNode::control_loop, this));
  }

  // Destructor handles safe shutdown
  ~DamiaoMotorNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Damiao Node...");
    disable_all();
  }

private:
  void init_commands() {
    FL_cmds_.fill({0, 0, 0, 0, 0}); FL_cmds_[2].q = ZERO_POS;
    FR_cmds_.fill({0, 0, 0, 0, 0}); FR_cmds_[2].q = ZERO_POS;
    BL_cmds_.fill({0, 0, 0, 0, 0}); BL_cmds_[2].q = ZERO_POS;
    BR_cmds_.fill({0, 0, 0, 0, 0}); BR_cmds_[2].q = ZERO_POS;
  }

  void disable_all() {
    if(dm_FL_) dm_FL_->disable();
    if(dm_FR_) dm_FR_->disable();
    if(dm_BL_) dm_BL_->disable();
    if(dm_BR_) dm_BR_->disable();
  }

  void command_callback(const damiao_hscant::msg::DmCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(dm_cmd_mutex_);
    for (size_t i = 0; i < msg->kp.size(); i++) {
      int leg = i / 3;
      int joint = i % 3;
      
      // Ensure we don't access out of bounds if message size is wrong
      if (leg > 3) continue;

      dm_cmd cmd = {msg->pos[i], msg->vel[i], msg->tau[i], msg->kp[i], msg->kd[i]};

      switch (leg) {
      case 0: FL_cmds_[joint] = cmd; break;
      case 1: FR_cmds_[joint] = cmd; break;
      case 2: BL_cmds_[joint] = cmd; break;
      case 3: BR_cmds_[joint] = cmd; break;
      default: break;
      }
    }
  }

  void control_loop() {
    // --- 1. SEND COMMANDS ---
    {
      std::lock_guard<std::mutex> lock(dm_cmd_mutex_);
      for (size_t i = 0; i < 3; i++) {
        float offset = (i == 2) ? ZERO_POS : 0.0f;

        dm_FL_->control_mit(*FL_M_[i], FL_cmds_[i].kp, FL_cmds_[i].kd,
                            FL_cmds_[i].q * FL_dir[i] - offset,
                            FL_cmds_[i].dq * FL_dir[i],
                            FL_cmds_[i].tau * FL_dir[i]);

        dm_FR_->control_mit(*FR_M_[i], FR_cmds_[i].kp, FR_cmds_[i].kd,
                            FR_cmds_[i].q * FR_dir[i] + offset,
                            FR_cmds_[i].dq * FR_dir[i],
                            FR_cmds_[i].tau * FR_dir[i]);

        dm_BL_->control_mit(*BL_M_[i], BL_cmds_[i].kp, BL_cmds_[i].kd,
                            BL_cmds_[i].q * BL_dir[i] - offset,
                            BL_cmds_[i].dq * BL_dir[i],
                            BL_cmds_[i].tau * BL_dir[i]);

        dm_BR_->control_mit(*BR_M_[i], BR_cmds_[i].kp, BR_cmds_[i].kd,
                            BR_cmds_[i].q * BR_dir[i] + offset,
                            BR_cmds_[i].dq * BR_dir[i],
                            BR_cmds_[i].tau * BR_dir[i]);
      }
    }

    // --- 2. READ STATE ---
    motor_state_msg_.pos = {
        FL_M_[0]->Get_Position(), FL_M_[1]->Get_Position(), FL_M_[2]->Get_Position() + ZERO_POS,
        FR_M_[0]->Get_Position(), FR_M_[1]->Get_Position(), FR_M_[2]->Get_Position() - ZERO_POS,
        BL_M_[0]->Get_Position(), BL_M_[1]->Get_Position(), BL_M_[2]->Get_Position() + ZERO_POS,
        BR_M_[0]->Get_Position(), BR_M_[1]->Get_Position(), BR_M_[2]->Get_Position() - ZERO_POS};

    motor_state_msg_.vel = {
        FL_M_[0]->Get_Velocity(), FL_M_[1]->Get_Velocity(), FL_M_[2]->Get_Velocity(),
        FR_M_[0]->Get_Velocity(), FR_M_[1]->Get_Velocity(), FR_M_[2]->Get_Velocity(),
        BL_M_[0]->Get_Velocity(), BL_M_[1]->Get_Velocity(), BL_M_[2]->Get_Velocity(),
        BR_M_[0]->Get_Velocity(), BR_M_[1]->Get_Velocity(), BR_M_[2]->Get_Velocity()};

    motor_state_msg_.tau = {
        FL_M_[0]->Get_tau(), FL_M_[1]->Get_tau(), FL_M_[2]->Get_tau(),
        FR_M_[0]->Get_tau(), FR_M_[1]->Get_tau(), FR_M_[2]->Get_tau(),
        BL_M_[0]->Get_tau(), BL_M_[1]->Get_tau(), BL_M_[2]->Get_tau(),
        BR_M_[0]->Get_tau(), BR_M_[1]->Get_tau(), BR_M_[2]->Get_tau()};

    // --- 3. TRANSFORM STATE ---
    for (size_t i = 0; i < Motor_dirs.size(); i++) {
      for (size_t j = 0; j < FL_dir.size(); j++) {
        if (Motor_dirs[i][j] == -1) {
          int index = i * 3 + j;
          motor_state_msg_.pos[index] = -motor_state_msg_.pos[index];
          motor_state_msg_.vel[index] = -motor_state_msg_.vel[index];
          motor_state_msg_.tau[index] = -motor_state_msg_.tau[index];
        }
      }
    }

    // --- 4. PUBLISH ---
    pub_->publish(motor_state_msg_);
  }

  // Member Variables
  std::unique_ptr<HSCanT::HSCanT> hscant_;
  std::unique_ptr<HSCanT::HSCanT_handler> FL_handler_, FR_handler_, BL_handler_, BR_handler_;
  
  std::unique_ptr<damiao::Motor_Control> dm_FL_, dm_FR_, dm_BL_, dm_BR_;
  std::array<std::shared_ptr<damiao::Motor>, 3> FL_M_, FR_M_, BL_M_, BR_M_;

  std::mutex dm_cmd_mutex_;
  std::array<dm_cmd, 3> FL_cmds_, FR_cmds_, BL_cmds_, BR_cmds_;

  damiao_hscant::msg::DmState motor_state_msg_;
  rclcpp::Subscription<damiao_hscant::msg::DmCommand>::SharedPtr sub_;
  rclcpp::Publisher<damiao_hscant::msg::DmState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DamiaoMotorNode>();

  // Use MultiThreadedExecutor to allow the timer (control loop) and 
  // subscriber (commands) to run in parallel threads safely.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
