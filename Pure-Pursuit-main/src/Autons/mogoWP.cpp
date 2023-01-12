#include "Autons/mogoWP.hpp"
#include "main.h"
#include "api.h"
#include "auton.hpp"
#include "config.hpp"
#include "movement.hpp"
#include "robot.hpp"
#include "odom.hpp"
#include "PID.hpp"
#include "initialize.hpp"

void route1(bool backwards)
{
  double finalX = 72.1;
  double finalY = 48.41;
  std::vector<CurvePoint> allPoints;
  CurvePoint start(pos_x, pos_y, 0, 0, 14, 5, 1);
  CurvePoint newPoint2(0, 48, 0, 0, 14, 5, 1);
  CurvePoint newPoint3(24, 48.1, 0, 0, 14, 5, 1);
  CurvePoint newPoint4(24.1, 0.5, 0, 0, 14, 5, 1);
  CurvePoint newPoint5(48.1, 0.1, 0, 0, 14, 5, 1);
  CurvePoint newPoint6(48.6, 48.12, 0, 0, 14, 5, 1);
  CurvePoint newPoint7(finalX, finalY, 0, 0, 14, 5, 1);
  allPoints.push_back(start);
  allPoints.push_back(newPoint2);
  allPoints.push_back(newPoint3);
  allPoints.push_back(newPoint4);
  allPoints.push_back(newPoint5);
  allPoints.push_back(newPoint6);
  allPoints.push_back(newPoint7);

  while (true)
  {
    if (sqrt(pow(finalX - pos_x, 2) + pow(finalY - pos_y, 2)) < 15)
    {
      arcMovementPID(finalX, finalY, backwards, false,false);
      stop();
      break;
    }
    followCurve(allPoints, 0, backwards);
  }
}

void route(double x, double y, bool backwards)
{
  double finalX = -22;
  double finalY = 82;
  std::vector<CurvePoint> allPoints;
  CurvePoint start(x, y, 0, 0, 8, 5, 1);
  CurvePoint newPoint3(finalX, finalY, 0, 0, 8, 5, 1);

  allPoints.push_back(start);
  allPoints.push_back(newPoint3);

  while (true)
  {
    if (sqrt(pow(finalX - pos_x, 2) + pow(finalY - pos_y, 2)) < 4)
    {
      std::cout << "BREAK";
      straightPID(finalX, finalY, 0.05);
      break;
    }
    followCurve(allPoints, 0, backwards);
  }
}

void mogoWP_auton()
{
  grabber.set_value(0);
  arcMovementPID(-28, 46.5, false, true, false); // make so cgrab and clamp :(
                                               // ddBackwards(-6000*40);
  grabber.set_value(1);
  clamp.set_value(true);
  arcMovementPID(-10, 31, true, false, false);
  stop();
  for(int i = 0; i < 20; i++){
    liftUp(10000);
    pros::delay(10);
  }
  liftStop();
  setLiftHold();
  turnPID(145, 2);
  maxSpeedPID = 12000;
  arcMovementPID(13.6, 0, false, false, false); // go to corner
  grabber.set_value(0);                 // drop goal in corner.
  setLiftBrake();
  arcMovementPID(-4.8, 18.6, true, false, false); // backup
  turnPID(270, 1.3);

  arcMovementPID(15, 18.6, true, false, true); // backup // need to make a break once clamped
  clamp.set_value(0);
  stop();
  maxSpeedPID = 9000;

  arcMovementPID(10, pos_y, false, false, false); // go fwd to lineup with rings
  turnPID(0, 2);
  for(int i = 0; i < 10; i++){
    liftUp(10000);
    pros::delay(10);
  }
  liftStop();
  int transmissionCounter = 0;
  // clampUp = !clampUp;
  // driveTrans.set_value(0);
  // intakeTrans.set_value(1); 
  sixMotorDrive = false;
  runIntake();
  maxSpeedPID = 5000;
  arcMovementPID(10, 50, false, false, false); // make so cgrab and clamp :(
  runIntake();
  maxSpeedPID = 10000;
  arcMovementPID(10, 10, true, false, false); // make so cgrab and clamp :(
  // driveTrans.set_value(1);
  // intakeTrans.set_value(0); 
  stop();
  stopIntake();
  clamp.set_value(1);
  // clampUp = !clampUp;
}

void routeforward(double x, double y, bool backwards)
{
  double finalX = 0.2;
  double finalY = 82;
  std::vector<CurvePoint> allPoints;
  CurvePoint start(x, y, 0, 0, 8, 5, 1);
  CurvePoint newPoint3(finalX, finalY, 0, 0, 8, 5, 1);

  allPoints.push_back(start);
  allPoints.push_back(newPoint3);

  while (true)
  {
    if (sqrt(pow(finalX - pos_x, 2) + pow(finalY - pos_y, 2)) < 4)
    {
      std::cout << "BREAK";
      straightPID(finalX, finalY, 0.05);
      break;
    }
    followCurve(allPoints, 0, backwards);
  }
}

void neutralGoalWP(){

}

void skillsAuto()
{
}
