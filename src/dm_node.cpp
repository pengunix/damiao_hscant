#include "HSCanT.h"
#include "damiao.h"
#include "dm_motor_ros/DmCommand.h"
#include "dm_motor_ros/DmState.h"
#include "ros/ros.h"
#include <csignal>
#include <iostream>

constexpr damiao::DM_Motor_Type MotorType = damiao::DM8009;
constexpr float ZERO_POS = 1.5707963267948;
constexpr std::array<int, 3> FL_dir = {1, 1, -1};
constexpr std::array<int, 3> FR_dir = {1, -1, 1};
constexpr std::array<int, 3> BL_dir = {-1, 1, -1};
constexpr std::array<int, 3> BR_dir = {-1, -1, 1};
constexpr std::array<std::array<int, 3>, 4> Motor_dirs = {FL_dir, FR_dir,
                                                          BL_dir, BR_dir};

struct dm_cmd {
  float q;
  float dq;
  float tau;
  float kp;
  float kd;
};

std::mutex dm_cmd_mutex;
std::array<dm_cmd, 3> FL_cmds;
std::array<dm_cmd, 3> FR_cmds;
std::array<dm_cmd, 3> BL_cmds;
std::array<dm_cmd, 3> BR_cmds;

dm_motor_ros::DmState motor_state_msg;

extern "C" void sigint_handler(int signal) { ROS_INFO("ROS Exited!"); ros::shutdown(); }

void MotorCmdCallback(const dm_motor_ros::DmCommandConstPtr &msg);

int main(int argc, char **argv) {
  ros::init(argc, argv, "dm_main");
  // ros::NodeHandle nodeHandle;
  auto nodeHandle = std::make_unique<ros::NodeHandle>();
  ros::Rate loop_rate(100);

  std::unique_ptr<HSCanT::HSCanT> hscant = std::make_unique<HSCanT::HSCanT>();

  if (!hscant->is_valid()) {
    LOGE("HSCanT initialization failed!");
    return -1;
  }
  std::unique_ptr<HSCanT::HSCanT_handler> FL_handler = hscant->export_handler(0);
  std::unique_ptr<HSCanT::HSCanT_handler> FR_handler = hscant->export_handler(1);
  std::unique_ptr<HSCanT::HSCanT_handler> BL_handler = hscant->export_handler(2);
  std::unique_ptr<HSCanT::HSCanT_handler> BR_handler = hscant->export_handler(3);

  std::shared_ptr<damiao::Motor> FL_M0 =
      std::make_shared<damiao::Motor>(MotorType, 0x01, 0x11);
  std::shared_ptr<damiao::Motor> FL_M1 =
      std::make_shared<damiao::Motor>(MotorType, 0x02, 0x12);
  std::shared_ptr<damiao::Motor> FL_M2 =
      std::make_shared<damiao::Motor>(MotorType, 0x03, 0x13);
  // damiao::Motor_Control dm_FL(FL_handler.get());
  std::unique_ptr<damiao::Motor_Control> dm_FL = std::make_unique<damiao::Motor_Control>(FL_handler.get());
  std::array<std::shared_ptr<damiao::Motor>, 3> FL_M = {FL_M0, FL_M1, FL_M2};
  dm_FL->addMotor({FL_M0, FL_M1, FL_M2});

  std::shared_ptr<damiao::Motor> FR_M0 =
      std::make_shared<damiao::Motor>(MotorType, 0x01, 0x11);
  std::shared_ptr<damiao::Motor> FR_M1 =
      std::make_shared<damiao::Motor>(MotorType, 0x02, 0x12);
  std::shared_ptr<damiao::Motor> FR_M2 =
      std::make_shared<damiao::Motor>(MotorType, 0x03, 0x13);
  //damiao::Motor_Control dm_FR(FR_handler.get());
  std::unique_ptr<damiao::Motor_Control> dm_FR = std::make_unique<damiao::Motor_Control>(FR_handler.get());
  std::array<std::shared_ptr<damiao::Motor>, 3> FR_M = {FR_M0, FR_M1, FR_M2};
  dm_FR->addMotor({FR_M0, FR_M1, FR_M2});

  std::shared_ptr<damiao::Motor> BL_M0 =
      std::make_shared<damiao::Motor>(MotorType, 0x01, 0x11);
  std::shared_ptr<damiao::Motor> BL_M1 =
      std::make_shared<damiao::Motor>(MotorType, 0x02, 0x12);
  std::shared_ptr<damiao::Motor> BL_M2 =
      std::make_shared<damiao::Motor>(MotorType, 0x03, 0x13);
  //damiao::Motor_Control dm_BL(BL_handler.get());
  std::unique_ptr<damiao::Motor_Control> dm_BL = std::make_unique<damiao::Motor_Control>(BL_handler.get());
  std::array<std::shared_ptr<damiao::Motor>, 3> BL_M = {BL_M0, BL_M1, BL_M2};
  dm_BL->addMotor({BL_M0, BL_M1, BL_M2});

  std::shared_ptr<damiao::Motor> BR_M0 =
      std::make_shared<damiao::Motor>(MotorType, 0x01, 0x11);
  std::shared_ptr<damiao::Motor> BR_M1 =
      std::make_shared<damiao::Motor>(MotorType, 0x02, 0x12);
  std::shared_ptr<damiao::Motor> BR_M2 =
      std::make_shared<damiao::Motor>(MotorType, 0x03, 0x13);
  std::array<std::shared_ptr<damiao::Motor>, 3> BR_M = {BR_M0, BR_M1, BR_M2};
  //damiao::Motor_Control dm_BR(BR_handler.get());
  std::unique_ptr<damiao::Motor_Control> dm_BR = std::make_unique<damiao::Motor_Control>(BR_handler.get());
  dm_BR->addMotor({BR_M0, BR_M1, BR_M2});

  // 这里获取到的是原始位置，只要不为未初始化的0都代表电机初始化完成
  sleep(1);
  dm_FL->enable();
  dm_FR->enable();
  dm_BL->enable();
  dm_BR->enable();

  ROS_INFO("=================================================");
  std::vector<float> init_pos = {
      FL_M0->Get_Position(), FL_M1->Get_Position(), FL_M2->Get_Position(),
      FR_M0->Get_Position(), FR_M1->Get_Position(), FR_M2->Get_Position(),
      BL_M0->Get_Position(), BL_M1->Get_Position(), BL_M2->Get_Position(),
      BR_M0->Get_Position(), BR_M1->Get_Position(), BR_M2->Get_Position()};
  for (int i = 0; i < init_pos.size(); i++) {
    ROS_INFO("Motor %d Initial Position: %.4f", i, init_pos[i]);
    if (fabs(init_pos[0]) == 0.0000) {
      ROS_ERROR("Motor %d may not be connected properly!", i);
      dm_FL->disable();
      dm_FR->disable();
      dm_BL->disable();
      dm_BR->disable();
      return 1;
    }
  }
  ROS_INFO("Damiao Motor Node Initialized.");
  ROS_INFO("=================================================");


  FL_cmds.fill({0, 0, 0, 0, 0});
  FL_cmds[2].q = ZERO_POS;
  FR_cmds.fill({0, 0, 0, 0, 0});
  FR_cmds[2].q = ZERO_POS;
  BL_cmds.fill({0, 0, 0, 0, 0});
  BL_cmds[2].q = ZERO_POS;
  BR_cmds.fill({0, 0, 0, 0, 0});
  BR_cmds[2].q = ZERO_POS;

  motor_state_msg.joint_names = {"FL_M0", "FL_M1", "FL_M2", "FR_M0",
                                 "FR_M1", "FR_M2", "BL_M0", "BL_M1",
                                 "BL_M2", "BR_M0", "BR_M1", "BR_M2"};

  boost::function<void(const dm_motor_ros::DmCommandConstPtr &)> callback =
      [&](const dm_motor_ros::DmCommandConstPtr &msg) -> void {
    std::lock_guard<std::mutex> lock(dm_cmd_mutex);
    for (int i = 0; i < msg->kp.size(); i++) {
      int leg = i / 3;
      int joint = i % 3;
      float q_cmd = msg->pos[i];
      float dq_cmd = msg->vel[i];
      float tau_cmd = msg->tau[i];
      float kp_cmd = msg->kp[i];
      float kd_cmd = msg->kd[i];
      switch (leg) {
      case 0:
        FL_cmds[joint] = {q_cmd, dq_cmd, tau_cmd, kp_cmd, kd_cmd};
        break;
      case 1:
        FR_cmds[joint] = {q_cmd, dq_cmd, tau_cmd, kp_cmd, kd_cmd};
        break;
      case 2:
        BL_cmds[joint] = {q_cmd, dq_cmd, tau_cmd, kp_cmd, kd_cmd};
        break;
      case 3:
        BR_cmds[joint] = {q_cmd, dq_cmd, tau_cmd, kp_cmd, kd_cmd};
        break;
      default:
        break;
      }
    }
  };
  ros::Subscriber MotorCmdSub = nodeHandle->subscribe("/dm_cmd", 128, callback);

  ros::Publisher MotorStatePub =
      nodeHandle->advertise<dm_motor_ros::DmState>("/dm_states", 128);

  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  while (ros::ok()) {
    if (std::signal(SIGINT, sigint_handler) == SIG_ERR) {
      return 1;
    }

    {
      std::lock_guard<std::mutex> lock(dm_cmd_mutex);
      for (int i = 0; i < FL_M.size(); i++) {
        float offset = 0;
        if (i == 2) {
          offset = ZERO_POS;
        }
        dm_FL->control_mit(*FL_M[i], FL_cmds[i].kp, FL_cmds[i].kd,
                          FL_cmds[i].q * FL_dir[i] - offset,
                          FL_cmds[i].dq * FL_dir[i],
                          FL_cmds[i].tau * FL_dir[i]);
        dm_FR->control_mit(*FR_M[i], FR_cmds[i].kp, FR_cmds[i].kd,
                          FR_cmds[i].q * FR_dir[i] + offset,
                          FR_cmds[i].dq * FR_dir[i],
                          FR_cmds[i].tau * FR_dir[i]);
        dm_BL->control_mit(*BL_M[i], BL_cmds[i].kp, BL_cmds[i].kd,
                          BL_cmds[i].q * BL_dir[i] - offset,
                          BL_cmds[i].dq * BL_dir[i],
                          BL_cmds[i].tau * BL_dir[i]);
        dm_BR->control_mit(*BR_M[i], BR_cmds[i].kp, BR_cmds[i].kd,
                          BR_cmds[i].q * BR_dir[i] + offset,
                          BR_cmds[i].dq * BR_dir[i],
                          BR_cmds[i].tau * BR_dir[i]);
      }
    }

    // 右侧大腿关节取相反数
    // 左侧小腿关节减去零位后取相反数
    // 速度、力矩同理，只取反即可
    // TODO(me): 封装状态数组以对应电机和索引，并完成姿态转换
    motor_state_msg.pos = {FL_M0->Get_Position(),
                           FL_M1->Get_Position(),
                           FL_M2->Get_Position() + ZERO_POS,
                           FR_M0->Get_Position(),
                           FR_M1->Get_Position(),
                           FR_M2->Get_Position() - ZERO_POS,
                           BL_M0->Get_Position(),
                           BL_M1->Get_Position(),
                           BL_M2->Get_Position() + ZERO_POS,
                           BR_M0->Get_Position(),
                           BR_M1->Get_Position(),
                           BR_M2->Get_Position() - ZERO_POS};

    motor_state_msg.vel = {
        FL_M0->Get_Velocity(), FL_M1->Get_Velocity(), FL_M2->Get_Velocity(),
        FR_M0->Get_Velocity(), FR_M1->Get_Velocity(), FR_M2->Get_Velocity(),
        BL_M0->Get_Velocity(), BL_M1->Get_Velocity(), BL_M2->Get_Velocity(),
        BR_M0->Get_Velocity(), BR_M1->Get_Velocity(), BR_M2->Get_Velocity()};

    motor_state_msg.tau = {
        FL_M0->Get_tau(), FL_M1->Get_tau(), FL_M2->Get_tau(), FR_M0->Get_tau(),
        FR_M1->Get_tau(), FR_M2->Get_tau(), BL_M0->Get_tau(), BL_M1->Get_tau(),
        BL_M2->Get_tau(), BR_M0->Get_tau(), BR_M1->Get_tau(), BR_M2->Get_tau()};
    for (int i = 0; i < Motor_dirs.size(); i++) {
      for (int j = 0; j < FL_dir.size(); j++) {
        if (Motor_dirs[i][j] == -1) {
          int index = i * 3 + j;
          motor_state_msg.pos[index] = -motor_state_msg.pos[index];
          motor_state_msg.vel[index] = -motor_state_msg.vel[index];
          motor_state_msg.tau[index] = -motor_state_msg.tau[index];
        }
      }
    }
    MotorStatePub.publish(motor_state_msg);
    loop_rate.sleep();
  }
  // Motors May not need to be reset
  dm_FL.reset();
  dm_FR.reset();
  dm_BL.reset();
  dm_BR.reset();
  FL_handler.reset();
  FR_handler.reset();
  BL_handler.reset();
  BR_handler.reset();
  hscant.reset();

  return 0;
}
