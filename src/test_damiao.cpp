#include "damiao.h"
#include "unistd.h"
#include <cmath>
#include <iostream>


int main() {

  damiao::Motor M1(damiao::DM8009, 0x01, 0x11);
  damiao::Motor_Control dm("/dev/ttyACM0", B1000000);

  dm.addMotor(&M1);
  dm.disable();
  usleep(200);

  dm.enable();

  for(;;) {
    dm.control_mit(M1, 5, 0.3, 0, 0, 0);

    std::cout<<"motor1--- POS:"<<M1.Get_Position()<<" VEL:"<<M1.Get_Velocity()<<" CUR:"<<M1.Get_tau()<<std::endl;
    sleep(1);
  }

  return 0;
}
