#include "main.h"
#include "PID.hpp"
#include "odom.hpp"
#include "api.h"
#include "config.hpp"
#include "robot.hpp"
#include "auton.hpp"

bool toggle = true;
bool toggle2 = true;
double displacementX;
double displacementY;
double headingGoal;
double headingStart;
double headingCont;
double headingCart;
double turnPreviousError;
double turnCount;
double errorCount;
double turnError;
double forwardPreviousError;
double distanceCount;
double distanceError;
double turnIntegral;
double turnKP; // turn correction proportional
double turnKI; // turn correction integral
double turnKD; // turn correction derivative
double distancePreviousError;
double xTarget;
double yTarget;

PID::PID(double kP, double kI, double kD, double target, double maxSpeed)
{ // PIDStraight
  p_kP = kP;
  p_kI = kI;
  p_kD = kD;
  p_maxSpeed = maxSpeed;
  p_target = target;
}

void PID::straightPID()
{ // This will have no correction... which is okay since the robot shouldn't mess up in the last 5-ish inches.. right?

  double leftStartingValue = leftRotationValue();   // in rotationUnits
  double rightStartingValue = rightRotationValue(); // in rotationUnits
  double previousError = 0;                         // this could cause a problem but only for the first iteration

  int thresholdTimer = 0;
  int failsafeTimer = 0;

  while (true)
  {
    double leftValue = (-leftStartingValue + leftRotationValue()) * (1.375) * 1.004 * M_PI / 180; // convert to inches
    double rightValue = (-rightStartingValue + rightRotationValue()) * (1.375) * 1.004 * M_PI / 180;
    double avgValue = (rightValue + leftValue) / 2;
    double error = p_target - avgValue;

    double derivative = error - previousError;

    double speed = error * p_kP + derivative * p_kD;

    left_move(-speed);
    right_move(speed);

    if (fabs(error) < 1)
    { // threshold
      thresholdTimer++;
    }
    else
    {
      thresholdTimer = 0;
    }
    if (thresholdTimer >= 20)
    {
      stop();
      break;
    }

    if (error - previousError < 0.3)
    {
      failsafeTimer++;
    }
    else
    {
      failsafeTimer = 0;
    }

    if (failsafeTimer >= 300)
    { // 3 second failsafe
      std::cout << "failsafe";
      stop();
      break;
    }

    previousError = error;
    odom();
    counter++;
    pros::delay(10);
  }
}

TURNPID::TURNPID(double kP, double kI, double kD, double targetTheta, double maxSpeed)
{
  p_kP = kP;
  p_kI = kI;
  p_kD = kD;
  p_targetTheta = targetTheta;
  p_maxSpeed = maxSpeed;
}

TURNPID::TURNPID(double kP, double kI, double kD, double x, double y, double maxSpeed)
{
  p_kP = kP;
  p_kI = kI;
  p_kD = kD;
  p_targetTheta = atan2f(x - pos_x, y - pos_y) * 180 / M_PI;
  p_maxSpeed = maxSpeed;
}

void TURNPID::turnPID()
{
  double thresholdTimer = 0;
  double failsafeTimer = 0;
  double previousError = 0;
  p_targetTheta = p_targetTheta * M_PI / 180;
  while (true)
  {
    double error = p_targetTheta - theta;
    error = atan2f(sinf(error), cosf(error)) * 180 / M_PI;
    double dertivative = error - previousError;

    if (fabs(error) < 1.5)
    { // threshold (degrees)
      thresholdTimer++;
    }
    else
    {
      thresholdTimer = 0;
    }
    if (thresholdTimer >= 20)
    {
      stop();
      break;
    }

    if (error - previousError < 0.3)
    {
      failsafeTimer++;
    }
    else
    {
      failsafeTimer = 0;
    }

    if (failsafeTimer >= 300)
    { // 3 second failsafe
      pros::lcd::print(7, "failsafe engaged");
      stop();
      break;
    }

    double speed = error * p_kP + dertivative * p_kD;
    if (speed > p_maxSpeed)
      speed = p_maxSpeed;

    left_move(-speed);
    right_move(-speed);

    previousError = error;
    odom();
    counter++;
    pros::delay(10);
  }
}
double normalize(const double value, const double start, const double end)
{
  const double width = end - start;         //
  const double offsetValue = value - start; // value relative to 0

  return (offsetValue - (floor(offsetValue / width) * width)) + start;
  // + start to reset back to start of original range
}

int STRAIGHTPIDKP = 1150; 
int DDSTRAIGHTMAX = 12000;
void ddStraightNoFail(double target, bool grab){
  //150000 = one mat.

  //	 target = target*150000/24;


  double reference = fwdRotation.get_position()*(1.0/100.0)*(M_PI/180)*(3.0/5.0) * (2.75/2);// determines inches
  // if(!sixMotorDrive){
  //   std::cout<<"4M drive"<< std::endl;
  //   kpee = 1600;
  // }
  double previousKP = STRAIGHTPIDKP;
  if(clamped() && !grabbed()){
    STRAIGHTPIDKP += 5;
  }
  if(grabbed() && !clamped()){
    STRAIGHTPIDKP += 5;
  }

  if(grabbed() && clamped()){
    STRAIGHTPIDKP += 7;
  }
  
  int escapeCounter = 0;
  double failsafe = 0;
  int failsafeCounter = 0;

  std::cout<<"ddStraight, KP: " << STRAIGHTPIDKP <<std::endl;

  while (true)
  {
    frontGrabberAuton();
    if(grab)
      if(grabbed()){
        stop();
        STRAIGHTPIDKP = previousKP;
        break;
      }

    double current = fwdRotation.get_position()*(1.0/100.0)*(M_PI/180) * (3.0/5.0) * (2.75/2);
    double error = target - (current - reference);
    double speed = error * STRAIGHTPIDKP;

    if(fabs(speed) > DDSTRAIGHTMAX){
      speed = DDSTRAIGHTMAX;
    }
    
    left_move(-speed);
    right_move(speed);
    std::cout << "Speed: " << speed <<  "Error: " << error << "fwd rotation " << current <<  std::endl;
    if (fabs(error) < 1)
    {
      escapeCounter++;
    }
    else
    {
      escapeCounter = 0;
    }

    if (escapeCounter >= 1)
    {
      stop();
      STRAIGHTPIDKP = previousKP;
      break;
    }
    pros::delay(10);
  }
}

void ddStraight(double target,bool grab)
{
  // 150000 = one mat.

  //	 target = target*150000/24;


  double reference = fwdRotation.get_position()*(1.0/100.0)*(M_PI/180)*(3.0/5.0) * (2.75/2);// determines inches
  // if(!sixMotorDrive){
  //   std::cout<<"4M drive"<< std::endl;
  //   kpee = 1600;
  // }
  double previousKP = STRAIGHTPIDKP;
  if(clamped() && !grabbed()){
    STRAIGHTPIDKP += 5;
  }
  if(grabbed() && !clamped()){
    STRAIGHTPIDKP += 5;
  }

  if(grabbed() && clamped()){
    STRAIGHTPIDKP += 7;
  }
  
  int escapeCounter = 0;
  double failsafe = 0;
  int failsafeCounter = 0;

  std::cout<<"ddStraight, KP: " << STRAIGHTPIDKP <<std::endl;

  while (true)
  {
    frontGrabberAuton();
    if(grab)
      if(grabbed()){
        stop();
        STRAIGHTPIDKP = previousKP;
        break;
      }

    double current = fwdRotation.get_position()*(1.0/100.0)*(M_PI/180) * (3.0/5.0) * (2.75/2);
    double error = target - (current - reference);
    double speed = error * STRAIGHTPIDKP;

    if(fabs(speed) > DDSTRAIGHTMAX){
      speed = DDSTRAIGHTMAX;
    }
    
    left_move(-speed);
    right_move(speed);
    std::cout << "Speed: " << speed <<  "Error: " << error << "fwd rotation " << current <<  std::endl;
    if (fabs(error) < 1)
    {
      escapeCounter++;
    }
    else
    {
      escapeCounter = 0;
    }

    if(escapeCounter > 1 && AUTO_ID == 99){
      stop();
      STRAIGHTPIDKP = previousKP;
      break;
    }

    if (escapeCounter >= 20)
    {
      stop();
      STRAIGHTPIDKP = previousKP;
      break;
    }

    if (failsafeCounter % 60 == 0)
    {
      if (fabs(failsafe - error) < 1)
      {
        STRAIGHTPIDKP = previousKP;
        std::cout << "broke" << std::endl;
        break;
      }

      failsafe = error;
    }
    failsafeCounter++;

    // odom(); counter++;
    pros::delay(10);
  }
}

void ddBackwardsNoFail(double target, bool clamp){

  int clampCounter  = 0;
  double reference = -fwdRotation.get_position()*(1.0/100.0)*(M_PI/180)*(3.0/5.0) * (2.75/2);// determines inches
  double previousKP = STRAIGHTPIDKP;
  if(clamped() && !grabbed()){
    STRAIGHTPIDKP += 5;
  }
  if(grabbed() && !clamped()){
    STRAIGHTPIDKP += 5;
  }

  if(grabbed() && clamped()){
    STRAIGHTPIDKP += 7;
  }
  int escapeCounter = 0;
  double failsafe = 0;
  int failsafeCounter = 0;

  double previousTheta = 0;
  std::cout<<"ddBackwards, KP: " << STRAIGHTPIDKP <<std::endl;

  while (true)
  {

    // clampAuton();
    if(clamp)
      if(clamped()){
        clampCounter++;
      }
      if(clampCounter > 25)
      {
        closeClamp();
      }
    
    if(clampCounter > 30){
      stop();  STRAIGHTPIDKP = previousKP;
      clampCounter = 0;
      break;
    }
    double current = -fwdRotation.get_position()*(1.0/100.0)*(M_PI/180) * (3.0/5.0) * (2.75/2);
    double error = target - (current - reference);
    double speed = error * STRAIGHTPIDKP;
    
    if(speed > 9000)
      speed = 9000;

    left_move(speed);
    right_move(-speed);
    std::cout << "Speed: " << speed <<  "Error: " << error << "fwd rotation " << current <<  std::endl;
    if (fabs(error) < 1)
    {
      escapeCounter++;
    }
    else
    {
      escapeCounter = 0;
    }

    if (escapeCounter >= 10)
    {
      // std::cout<<"done";
      stop();    STRAIGHTPIDKP = previousKP;

      break;
    }
    
    previousTheta = theta;
    
    // odom();  counter++;

    pros::delay(10);
  }
}
void ddBackwards(double target, bool clamp)
{

  int clampCounter  = 0;
  double reference = -fwdRotation.get_position()*(1.0/100.0)*(M_PI/180)*(3.0/5.0) * (2.75/2);// determines inches
  double previousKP = STRAIGHTPIDKP;
  if(clamped() && !grabbed()){
    STRAIGHTPIDKP += 5;
  }
  if(grabbed() && !clamped()){
    STRAIGHTPIDKP += 5;
  }

  if(grabbed() && clamped()){
    STRAIGHTPIDKP += 7;
  }
  int escapeCounter = 0;
  double failsafe = 0;
  int failsafeCounter = 0;

  double previousTheta = 0;
  std::cout<<"ddBackwards, KP: " << STRAIGHTPIDKP <<std::endl;

  while (true)
  {

    // clampAuton();
    if(clamp)
      if(clamped()){
        clampCounter++;
      }
      if(clampCounter > 25)
      {
        closeClamp();
      }
    
    if(clampCounter > 30){
      stop();  STRAIGHTPIDKP = previousKP;
      clampCounter = 0;
      break;
    }
    double current = -fwdRotation.get_position()*(1.0/100.0)*(M_PI/180) * (3.0/5.0) * (2.75/2);
    double error = target - (current - reference);
    double speed = error * STRAIGHTPIDKP;
    
    if(speed > 9000)
      speed = 9000;

    left_move(speed);
    right_move(-speed);
    std::cout << "Speed: " << speed <<  "Error: " << error << "fwd rotation " << current <<  std::endl;
    if (fabs(error) < 1)
    {
      escapeCounter++;
    }
    else
    {
      escapeCounter = 0;
    }

    if (escapeCounter >= 20)
    {
      // std::cout<<"done";
      stop();    STRAIGHTPIDKP = previousKP;

      break;
    }
    if (failsafeCounter % 60 == 0)
    {
      if (fabs(failsafe - error) < 1)
      {
         STRAIGHTPIDKP = previousKP;

        // std::cout << "broke" << std::endl;
        break;
      }

      failsafe = error;
    }


    previousTheta = theta;
    failsafeCounter++;
    // odom();  counter++;

    pros::delay(10);
  }
}


//5.8, 13 - 33.8,13 - 36, -2.6, -26, -50(slow) 