#include "vex.h"
#include "cmath"
//#include "robot-config.cpp"

using namespace vex;

competition Competition;
brain Brain;
controller Controller1 = controller (primary);

//Front Intake
motor IntakeRBottom = motor (PORT18);
motor IntakeLBottom = motor (PORT13);

//Top Intake
motor IntakeLTop = motor (PORT2);
motor IntakeRTop = motor (PORT10);

//Drive Base
motor DriveRFront = motor (PORT20);
motor DriveRBack = motor (PORT19);
motor DriveLFront = motor (PORT11);
motor DriveLBack = motor (PORT12);

//Sensors
encoder DistanceEncoderLeft = encoder (Brain.ThreeWirePort.A);
encoder DistanceEncoderRight = encoder (Brain.ThreeWirePort.C);
line BallSensor = line (Brain.ThreeWirePort.E);
inertial InertialSensor = inertial (PORT14);
limit LimitBall = limit (Brain.ThreeWirePort.H);

void DriveForwards (int MaxVelocity, int Distance)
{
  DistanceEncoderLeft.setPosition (0, rotationUnits::deg);
  DistanceEncoderRight.setPosition (0, rotationUnits::deg);
  double Pi = 3.1415926535;
  int Velocity = 10;
  double P = 1.1;
  int Difference = 10;
  task::sleep (100);
  while ((2.75 * Pi * (DistanceEncoderLeft.position(rotationUnits::deg) / 360)) < Distance)
  {
      if (Velocity < MaxVelocity)
      {
        Velocity = Velocity * P;
      }

      if (DistanceEncoderLeft.position(rotationUnits::deg) < DistanceEncoderRight.position(rotationUnits::deg) - 5)
      {
        DriveRFront.spin(directionType::fwd, -Velocity, velocityUnits::pct);
        DriveRBack.spin(directionType::fwd, -Velocity, velocityUnits::pct);
        DriveLFront.spin(directionType::fwd, Velocity - Difference, velocityUnits::pct);
        DriveLBack.spin(directionType::fwd, Velocity - Difference, velocityUnits::pct);
      }
      else if (DistanceEncoderLeft.position(rotationUnits::deg) - 5 > DistanceEncoderRight.position(rotationUnits::deg))
      {
        DriveRFront.spin(directionType::fwd, -Velocity + Difference, velocityUnits::pct);
        DriveRBack.spin(directionType::fwd, -Velocity + Difference, velocityUnits::pct);
        DriveLFront.spin(directionType::fwd, Velocity, velocityUnits::pct);
        DriveLBack.spin(directionType::fwd, Velocity, velocityUnits::pct);
      }
      else
      {
        DriveRFront.spin(directionType::fwd, -Velocity, velocityUnits::pct);
        DriveRBack.spin(directionType::fwd, -Velocity, velocityUnits::pct);
        DriveLFront.spin(directionType::fwd, Velocity, velocityUnits::pct);
        DriveLBack.spin(directionType::fwd, Velocity, velocityUnits::pct);
      }

      if (LimitBall.pressing() == false)
      {
        //IntakeLBottom.spin(directionType::fwd, 100, velocityUnits::pct);
        //IntakeRBottom.spin(directionType::fwd, -100, velocityUnits::pct);
      }
      else
      {
        IntakeLBottom.stop(brakeType::coast);
        IntakeRBottom.stop(brakeType::coast);
      }
      task::sleep (100);
  }
}

void DriveBackwards (int MaxVelocity, int Distance)
{
  DistanceEncoderLeft.setPosition (0, rotationUnits::deg);
  DistanceEncoderRight.setPosition (0, rotationUnits::deg);
  double Pi = 3.1415926535;
  int Velocity = 10;
  double P = 1.1;
  task::sleep (100);
  while ((2.75 * Pi * (-DistanceEncoderLeft.position(rotationUnits::deg) / 360)) < Distance)
  {
      if (Velocity < MaxVelocity)
      {
        Velocity = Velocity * P;
      }

      DriveRFront.spin(directionType::fwd, Velocity, velocityUnits::pct);
      DriveRBack.spin(directionType::fwd, Velocity, velocityUnits::pct);
      DriveLFront.spin(directionType::fwd, -Velocity, velocityUnits::pct);
      DriveLBack.spin(directionType::fwd, -Velocity, velocityUnits::pct);
      task::sleep (100);
  }
}

void TurnRightt (int MaxVelocity, int Distance)
{
  DistanceEncoderLeft.setPosition (0, rotationUnits::deg);
  DistanceEncoderRight.setPosition (0, rotationUnits::deg);
  double Pi = 3.1415926535;
  int Velocity = 10;
  double P = 1.1;
  task::sleep (100);
  while ((2.75 * Pi * (-DistanceEncoderLeft.position(rotationUnits::deg) / 360)) < Distance)
  {
      if (Velocity < MaxVelocity)
      {
        Velocity = Velocity * P;
      }

      DriveRFront.spin(directionType::fwd, Velocity, velocityUnits::pct);
      DriveRBack.spin(directionType::fwd, Velocity, velocityUnits::pct);
      DriveLFront.spin(directionType::fwd, Velocity, velocityUnits::pct);
      DriveLBack.spin(directionType::fwd, Velocity, velocityUnits::pct);
      task::sleep (100);
  }
}

void TurnLeft (int MaxVelocity, int Distance)
{
  DistanceEncoderLeft.setPosition (0, rotationUnits::deg);
  DistanceEncoderRight.setPosition (0, rotationUnits::deg);
  double Pi = 3.1415926535;
  int Velocity = 10;
  double P = 1.1;
  task::sleep (100);
  while ((2.75 * Pi * (-DistanceEncoderLeft.position(rotationUnits::deg) / 360)) < Distance)
  {
      if (Velocity < MaxVelocity)
      {
        Velocity = Velocity * P;
      }

      DriveRFront.spin(directionType::fwd, -Velocity, velocityUnits::pct);
      DriveRBack.spin(directionType::fwd, -Velocity, velocityUnits::pct);
      DriveLFront.spin(directionType::fwd, -Velocity, velocityUnits::pct);
      DriveLBack.spin(directionType::fwd, -Velocity, velocityUnits::pct);
      task::sleep (100);
  }
}

void DriveHold ()
{
  DriveRFront.stop(brakeType::hold);
  DriveRBack.stop(brakeType::hold);
  DriveLFront.stop(brakeType::hold);
  DriveLBack.stop(brakeType::hold);
}

void pre_auton() 
{
  vexcodeInit();
}

void autonomous() 
{
  TurnLeft(50, 20);
  DriveHold();
}

void usercontrol() 
{
  //True False Code
  //bool IntakeIn = true;
  //bool ToggleIntakeIn = false; 
  while (true) 
  {
    double Pi = 3.1415926535;
    double EncoderRight;
    double EncoderLeft;
    double PreviousEncoderRight = 0;
    double PreviousEncoderLeft = 0;
    double EncoderChangeRight;
    double EncoderChangeLeft;
    double DR = 2.15625;
    double DL = 2.15625;
    double Angle;
    double PreviousAngle = 0;
    double Rad = Pi/180;
    double Deg = 180/Pi;

    EncoderRight = DistanceEncoderRight.position(rotationUnits::deg);
    EncoderLeft = DistanceEncoderLeft.position(rotationUnits::deg);
    EncoderChangeRight = EncoderRight - PreviousEncoderRight;
    EncoderChangeLeft = EncoderLeft - PreviousEncoderLeft;

    Angle = PreviousAngle + ((((EncoderRight - EncoderLeft) / 360) * 2.75 * Pi)/(DR + DL)) * Deg;

    PreviousEncoderRight = EncoderRight;
    PreviousEncoderLeft = EncoderLeft;
    PreviousAngle = Angle;
    task::sleep (100);
    

    // Sensor Values
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor (1, 1);
    Brain.Screen.print ("Distance Encoder Left: %.0f Degrees", DistanceEncoderLeft.position(rotationUnits::deg));
    Brain.Screen.setCursor (2, 1);
    Brain.Screen.print ("Distance Encoder Right: %.0f Degrees", DistanceEncoderRight.position(rotationUnits::deg));
    Brain.Screen.setCursor (3, 1);
    Brain.Screen.print ("Ball Sensor: %d Percent", BallSensor.value(percentUnits::pct));
    Brain.Screen.setCursor (4, 1);
    Brain.Screen.setCursor (5, 1);
    Brain.Screen.print ("Ball Sensor: %f Percent", Angle);
    Brain.Screen.render();
    //

    if (Controller1.ButtonR2.pressing() == true)
    {
      IntakeRBottom.spin (directionType::fwd, 100, velocityUnits::pct);
      IntakeLBottom.spin (directionType::fwd, -100, velocityUnits::pct);
      //IntakeIn = false;
    }
    else if (Controller1.ButtonR1.pressing() == true)
    {
      IntakeRBottom.spin (directionType::fwd, -100, velocityUnits::pct);
      IntakeLBottom.spin (directionType::fwd, 100, velocityUnits::pct);
    }
    else
    {
      IntakeRBottom.stop (brakeType::coast);
      IntakeLBottom.stop (brakeType::coast);
    }
    //

    if (Controller1.ButtonL1.pressing() == true)
    {
      IntakeRTop.spin (directionType::fwd, 100, velocityUnits::pct);
      IntakeLTop.spin (directionType::fwd, -100, velocityUnits::pct);
      //IntakeIn = false;
    }
    else if (Controller1.ButtonL2.pressing() == true)
    {
      IntakeRTop.spin (directionType::fwd, -100, velocityUnits::pct);
      IntakeLTop.spin (directionType::fwd, 100, velocityUnits::pct);
    }
    else
    {
      IntakeRTop.stop (brakeType::coast);
      IntakeLTop.stop (brakeType::coast);
    }

    //Drive Base
    if (abs(Controller1.Axis3.position(percentUnits::pct)) + abs(Controller1.Axis4.position(percentUnits::pct)) > 10)
    { 
      DriveRFront.spin (directionType::fwd, -Controller1.Axis3.position(percentUnits::pct) + Controller1.Axis4.position(percentUnits::pct), velocityUnits::pct);
      DriveRBack.spin (directionType::fwd, -Controller1.Axis3.position(percentUnits::pct) + Controller1.Axis4.position(percentUnits::pct), velocityUnits::pct);
      DriveLFront.spin (directionType::fwd, Controller1.Axis3.position(percentUnits::pct) +Controller1.Axis4.position(percentUnits::pct), velocityUnits::pct);
      DriveLBack.spin (directionType::fwd, Controller1.Axis3.position(percentUnits::pct) + Controller1.Axis4.position(percentUnits::pct), velocityUnits::pct);
    }
    else
    {
      DriveRFront.stop (brakeType::coast);
      DriveRBack.stop (brakeType::coast);
      DriveLFront.stop (brakeType::coast);
      DriveLBack.stop (brakeType::coast);
    }

    if (Controller1.ButtonA.pressing() == true)
    {
      DistanceEncoderLeft.setRotation (0, rotationUnits::deg);
    }
  }
}

int main() 
{
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
}