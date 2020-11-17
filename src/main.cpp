#include "vex.h"
//#include "robot-config.cpp"

using namespace vex;

competition Competition;
brain Brain;
controller Controller1 = controller(primary);
motor MotorIntakeR = motor (PORT9);
motor MotorIntakeL = motor (PORT7);
motor MotorR = motor (PORT8);
motor MotorL = motor (PORT2);

void pre_auton() 
{
  vexcodeInit();
}

void autonomous() 
{

}

void usercontrol() 
{
  while (true) 
  {
    if (Controller1.ButtonR1.pressing() == true)
    {
      MotorR.spin (directionType::fwd, 100, velocityUnits::pct);
      MotorL.spin (directionType::fwd, -100, velocityUnits::pct);
    }
    else if (Controller1.ButtonR2.pressing() == true)
    {
      MotorR.spin (directionType::fwd, -100, velocityUnits::pct);
      MotorL.spin (directionType::fwd, 100, velocityUnits::pct);
    }
    else
    {
      MotorR.stop (brakeType::coast);
      MotorL.stop (brakeType::coast);
    }

    if (Controller1.ButtonL1.pressing() == true)
    {
      MotorIntakeR.spin (directionType::fwd, 100, velocityUnits::pct);
      MotorIntakeL.spin (directionType::fwd, -100, velocityUnits::pct);
    }
    else if (Controller1.ButtonL2.pressing() == true)
    {
      MotorIntakeR.spin (directionType::fwd, -100, velocityUnits::pct);
      MotorIntakeL.spin (directionType::fwd, 100, velocityUnits::pct);
    }
    else
    {
      MotorIntakeR.stop (brakeType::coast);
      MotorIntakeL.stop (brakeType::coast);
    }
  }
}


int main() 
{
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
}