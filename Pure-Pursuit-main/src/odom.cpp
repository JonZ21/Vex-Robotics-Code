#include "odom.hpp"
#include "main.h"
#include "config.hpp"
#include "initialize.hpp"
#include "display/lvgl.h"

double innerL()
{
  double angle = fmod(imuLeft.get_rotation() * 1.034 * 0.9947, 360);
  while (angle < 0)
  {
    angle += 360;
  }
  while (angle > 360)
  {
    angle -= 360;
  }
  return angle;
}
double innerR()
{
  double angle = fmod(imuRight.get_rotation() * 1.0056, 360);
  while (angle < 0)
  {
    angle += 360;
  }
  while (angle > 360)
  {
    angle -= 360;
  }
  return angle;
}

double innerAVG()
{
  return (innerR() + innerL()) / 2;
}
int leftRotationValue()
{
  return leftRotation.get_position() * 3 / 500;
}

int rightRotationValue()
{
  return -rightRotation.get_position() * 3 / 500;
}
int centerRotationValue()
{
  return centerRotation.get_position() * 3 / 500;
}

int fwdRotationValue(){ // returns the degrees that the tracker has travelled
  return fwdRotation.get_position() * 3 / 500;
}

int sideRotationValue(){  // smae as above
  return -sideRotation.get_value();
}

int counter(0);
double currentLeft(0), lastLeft(0), deltaLeft(0);
double currentRight(0), lastRight(0), deltaRight(0);
double currentCenter(0), lastCenter(0), deltaCenter(0);
double theory(0), deltaTheory(0);
double theta(0), lastTheta(0), deltaTheta(0);
double rotationTheta(0), totalRotationTheta(0);
double deltaX(0), deltaY(0);
double pos_x(0), pos_y(0);
pros::Mutex mutex;

void odom()
{
  // Get the values of the inertials (radians)

  pros::c::gps_status_s_t gpsBackStatus = gpsBack.get_status();
  pros::c::gps_status_s_t gpsSideStatus = gpsSide.get_status();
  //***   PHASE 1 - GETTING ANGLE FROM IMU (every 30ms)
  double x = (cos(innerL() * M_PI / 180 + M_PI) + cos(innerR() * M_PI / 180 + M_PI)) / 2;
  double y = (sin(innerL() * M_PI / 180 + M_PI) + sin(innerR() * M_PI / 180 + M_PI)) / 2;

  // IDK HOW THIS WORKS !! works the same as modulus (gets the remainder)
  if (fmod(counter, 3) < 1)
  {
    theta = std::abs(atan2f(y, x) + M_PI); // theta is in radians
  }

  //***    PHASE 2 - GETTING 10ms updates with encoders

  currentLeft = leftRotationValue() * (1.375) * 1.004 * M_PI / 180;
  currentRight = rightRotationValue() * (1.375) * 1.004 * M_PI / 180;
  currentCenter = centerRotationValue() * (1.375) * 1.004 * M_PI / 180;
  rotationTheta = ((deltaLeft - deltaRight) / 14.375);
  rotationTheta = ((deltaLeft - deltaRight) / 14.375);

  // update the changes
  deltaLeft = currentLeft - lastLeft;
  deltaRight = currentRight - lastRight;
  deltaCenter = currentCenter - lastCenter;
  deltaTheta = theta - lastTheta;

  // Theory tracker
  deltaTheory = (deltaLeft - deltaRight) * 1.056 * 24.1 / 1.61 * (1.375) * M_PI / 180;
  theory += deltaTheory;
  totalRotationTheta += rotationTheta;
  //
  // char buffer[300];
  // sprintf(buffer, "LEFT: %F\n RIGHT: %f\nCENTER: %f\nTHEORY: %f\n THETA: %f\nLEFT_T: %f\nRIGHT_T: %f\nX: %f\nY: %f\n", currentLeft, currentRight, currentCenter, theory, theta * 180 / M_PI,innerL(),innerR(), pos_x, pos_y);
  // lv_label_set_text(infoDisplay, buffer);

  // sprintf(buffer, "BACK_X: %F\n BACK_Y: %f\nSIDE_X: %f\nSIDE_Y: %f", gpsBackStatus.x, gpsBackStatus.y, gpsSideStatus.x, gpsSideStatus.y);
  // lv_label_set_text(infoDisplay2, buffer);
  // delta Position
  deltaX = (((deltaLeft + deltaRight) / 2 * 1.0f * -sinf(-theta)) - ((deltaCenter + deltaTheory) * 1.0f * -cosf(-theta)));
  deltaY = (((deltaLeft + deltaRight) / 2 * 1.0f * cosf(-theta)) - ((deltaCenter + deltaTheory) * .98f * -sinf(-theta)));

  // Updating position
  pos_x = pos_x + deltaX;
  pos_y += deltaY;

  lastLeft = currentLeft;
  lastRight = currentRight;
  lastCenter = currentCenter;
  lastTheta = theta;
}


double currentForwards, previosuForwards, deltaFowards, deltaTheory2,theory2;
double currentOTheta, deltaOTheta, lastOTheta;
void twoSensorOdom(){
  double thetaNoCap = (imuLeft.get_rotation() + imuRight.get_rotation())/2;
  //***   PHASE 1 - GETTING ANGLE FROM IMU (every 30ms)
  double x = (cos(innerL() * M_PI / 180 + M_PI) + cos(innerR() * M_PI / 180 + M_PI)) / 2;
  double y = (sin(innerL() * M_PI / 180 + M_PI) + sin(innerR() * M_PI / 180 + M_PI)) / 2;

  // IDK HOW THIS WORKS !! works the same as modulus (gets the remainder)
  if (fmod(counter, 3) < 1)
  {
    theta = std::abs(atan2f(y, x) + M_PI); // theta is in radians
  }

  //***    PHASE 2 - GETTING 10ms updates with encoders

  currentForwards = fwdRotationValue() * (1.375) * 1.004 * M_PI / 180;
  currentCenter =  sideRotationValue() * (1.375) * 1.004 * M_PI / 180;
  currentOTheta = thetaNoCap;
  rotationTheta = ((deltaLeft - deltaRight) / 14.375);
  rotationTheta = ((deltaLeft - deltaRight) / 14.375);

  // update the changes
  deltaFowards = currentForwards - previosuForwards;
  deltaCenter = currentCenter - lastCenter;
  deltaTheta = theta - lastTheta;
  deltaOTheta = currentOTheta - lastOTheta;

  // Theory tracker
  deltaTheory = deltaOTheta*1.09*0.79*1.115*1.413*0.9566*.9436*1.066*.867*2.26*0.0677*0.8366*1.069*1.16*0.5611*.927/1.33/1.307;
  // right turn 
  deltaTheory2 = deltaOTheta*1.35*0.103*0.9566*.9436*1.066*.867*2.26*0.0677*0.8366*1.069*1.16*0.5611*.927/1.33/1.307;
  theory += deltaTheory;
  theory2 += deltaTheory2;
  totalRotationTheta += rotationTheta;
  //
  char buffer[300];
  sprintf(buffer, "Forwad:  %f\nCENTER: %f\nTHEORY: %f\nTHEORY2: %f\nTHETA: %f\nTHETA_NOCAP: %f\nX: %f\nY: %f\n", currentForwards, currentCenter, theory,theory2, theta * 180 / M_PI,thetaNoCap, pos_x, pos_y);
  lv_label_set_text(infoDisplay, buffer);

 sprintf(buffer, "RF: %f\nRM: %f\nRB: %f\nLF: %f\nLM: %f\nLB: %f\n", rightF.get_power(),rightM.get_power(),rightB.get_power(),leftF.get_power(),leftM.get_power(),leftB.get_power());
  lv_label_set_text(infoDisplay2, buffer);

  deltaX = (((deltaFowards - deltaTheory2) * 1.0f * -sinf(-theta)) - ((deltaCenter - deltaTheory) * 1.0f * -cosf(-theta)));
  deltaY = (((deltaFowards - deltaTheory2) * 1.0f * cosf(-theta)) - ((deltaCenter - deltaTheory) * .98f * -sinf(-theta)));

  // Updating position
  pos_x = pos_x + deltaX;
  pos_y += deltaY;

  previosuForwards = currentForwards;
  lastCenter = currentCenter;
  lastOTheta = currentOTheta;
  lastTheta = theta;
}

bool INTAKEOVERRIDE = false;
bool caught;
int caughtTimer = 0;
bool outake;
int outakeTimer = 0;

bool NOINTAKE = false;

void odomTask(void *param)
{
  
  while (true)
  {
    mutex.take(10);

    if(!NOINTAKE){
      if(intake.get_torque() >= 0.35){
            caughtTimer++;
          } else{
            caughtTimer = 0;
          }
          if(caughtTimer > 50){
            caught = true;
          }

          if(caught){
            outakeTimer++;
          }
          if(outakeTimer > 100){
            caught = false;
          }

          if(caught){
            intake.move_voltage(-12000);  
          }else if(liftPot.get_position() > 13000 && TILTUP){
            runIntake();
          } else if (INTAKEOVERRIDE){}
          else{
            stopIntake();
          }
    }
   
     double thetaNoCap = (imuLeft.get_rotation() + imuRight.get_rotation())/2;
  //***   PHASE 1 - GETTING ANGLE FROM IMU (every 30ms)
  double x = (cos(innerL() * M_PI / 180 + M_PI) + cos(innerR() * M_PI / 180 + M_PI)) / 2;
  double y = (sin(innerL() * M_PI / 180 + M_PI) + sin(innerR() * M_PI / 180 + M_PI)) / 2;

  // IDK HOW THIS WORKS !! works the same as modulus (gets the remainder)
  if (fmod(counter, 3) < 1)
  {
    theta = std::abs(atan2f(y, x) + M_PI); // theta is in radians
  }

  //***    PHASE 2 - GETTING 10ms updates with encoders

  currentForwards = fwdRotationValue() * (1.375) * 1.004 * M_PI / 180;
  currentCenter =  sideRotationValue() * (1.375) * 1.004 * M_PI / 180;
  currentOTheta = thetaNoCap;
  rotationTheta = ((deltaLeft - deltaRight) / 14.375);
  rotationTheta = ((deltaLeft - deltaRight) / 14.375);

  // update the changes
  deltaFowards = currentForwards - previosuForwards;
  deltaCenter = currentCenter - lastCenter;
  deltaTheta = theta - lastTheta;
  deltaOTheta = currentOTheta - lastOTheta;

  // Theory tracker
  deltaTheory = deltaOTheta*1.09*0.79*1.115*1.413*0.9566*.9436*1.066*.867*2.26*0.0677*0.8366*1.069*1.16*0.5611*.927/1.33/1.307;
  // right turn 
  deltaTheory2 = deltaOTheta*1.35*0.103*0.9566*.9436*1.066*.867*2.26*0.0677*0.8366*1.069*1.16*0.5611*.927/1.33/1.307;
  theory += deltaTheory;
  theory2 += deltaTheory2;
  totalRotationTheta += rotationTheta;
  //
  char buffer[300];
  sprintf(buffer, "Forwad:  %f\nCENTER: %f\nTHEORY: %f\nTHEORY2: %f\nTHETA: %f\nTHETA_NOCAP: %f\nX: %f\nY: %f\n", currentForwards, currentCenter, theory,theory2, theta * 180 / M_PI,thetaNoCap, pos_x, pos_y);
  lv_label_set_text(infoDisplay, buffer);

  sprintf(buffer, "RF: %f\nRM: %f\nRB: %f\nLF: %f\nLM: %f\nLB: %f\n", rightF.get_power(),rightM.get_power(),rightB.get_power(),leftF.get_power(),leftM.get_power(),leftB.get_power());
  lv_label_set_text(infoDisplay2, buffer);

  deltaX = (((deltaFowards - deltaTheory2) * 1.0f * -sinf(-theta)) - ((deltaCenter - deltaTheory) * 1.0f * -cosf(-theta)));
  deltaY = (((deltaFowards - deltaTheory2) * 1.0f * cosf(-theta)) - ((deltaCenter - deltaTheory) * .98f * -sinf(-theta)));

  // Updating position
  pos_x = pos_x + deltaX;
  pos_y += deltaY;

  previosuForwards = currentForwards;
  lastCenter = currentCenter;
  lastOTheta = currentOTheta;
  lastTheta = theta;
  
    mutex.give(); //  anything that some other thread could use should be done before giving the mutex.
    pros::delay(10);
  }
}