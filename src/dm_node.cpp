#include "damiao.h"
#include <iostream>
#include <csignal>

constexpr damiao::DM_Motor_Type MotorType = damiao::DM8009;

bool running = true;
extern "C" void sigint_handler(int signal) {
    running = false;
}

int main() {
  std::shared_ptr<damiao::Motor> FL_M0 =
  std::make_shared<damiao::Motor>(MotorType, 0x01, 0x11);
  std::shared_ptr<damiao::Motor> FL_M1 =
  std::make_shared<damiao::Motor>(MotorType, 0x02, 0x12);
  std::shared_ptr<damiao::Motor> FL_M2 =
  std::make_shared<damiao::Motor>(MotorType, 0x03, 0x13);
  damiao::Motor_Control dm_FL("/dev/ttyACM0", B921600);
  dm_FL.addMotor({FL_M0, FL_M1, FL_M2});

  std::shared_ptr<damiao::Motor> FR_M0 =
      std::make_shared<damiao::Motor>(MotorType, 0x01, 0x11);
  std::shared_ptr<damiao::Motor> FR_M1 =
      std::make_shared<damiao::Motor>(MotorType, 0x02, 0x12);
  std::shared_ptr<damiao::Motor> FR_M2 =
      std::make_shared<damiao::Motor>(MotorType, 0x03, 0x13);
  damiao::Motor_Control dm_FR("/dev/ttyACM1", B921600);
  dm_FR.addMotor({FR_M0, FR_M1, FR_M2});

  std::shared_ptr<damiao::Motor> BL_M0 =
      std::make_shared<damiao::Motor>(MotorType, 0x01, 0x11);
  std::shared_ptr<damiao::Motor> BL_M1 =
      std::make_shared<damiao::Motor>(MotorType, 0x02, 0x12);
  std::shared_ptr<damiao::Motor> BL_M2 =
      std::make_shared<damiao::Motor>(MotorType, 0x03, 0x13);
  damiao::Motor_Control dm_BL("/dev/ttyACM2", B921600);
  dm_BL.addMotor({BL_M0, BL_M1, BL_M2});

  std::shared_ptr<damiao::Motor> BR_M0 =
      std::make_shared<damiao::Motor>(MotorType, 0x01, 0x11);
  std::shared_ptr<damiao::Motor> BR_M1 =
      std::make_shared<damiao::Motor>(MotorType, 0x02, 0x12);
  std::shared_ptr<damiao::Motor> BR_M2 =
      std::make_shared<damiao::Motor>(MotorType, 0x03, 0x13);
  damiao::Motor_Control dm_BR("/dev/ttyACM3", B921600);
  dm_BR.addMotor({BR_M0, BR_M1, BR_M2});
  // TODO(me): 打印初始化总结，各个电机的初始化状态

  sleep(1);
  dm_FL.enable();
  dm_FR.enable();
  dm_BL.enable();
  dm_BR.enable();

  while (running) {
    if (std::signal(SIGINT, sigint_handler) == SIG_ERR) {
        return 1;
    }
    // 这里用来刷新电机数据，refresh motor data不好用
    dm_FL.enable();
    dm_FR.enable();
    dm_BL.enable();
    dm_BR.enable();

    std::vector<float> pos{
        FL_M0->Get_Position(), FL_M1->Get_Position(), FL_M2->Get_Position(),
        FR_M0->Get_Position(), FR_M1->Get_Position(), FR_M2->Get_Position(),
        BL_M0->Get_Position(), BL_M1->Get_Position(), BL_M2->Get_Position(),
        BR_M0->Get_Position(), BR_M1->Get_Position(), BR_M2->Get_Position()};
    for (const auto &p : pos) {
      std::cout << p << "\t";
    }
    std::cout << std::endl;
    usleep(10000);
  }

  return 0;
}