#include "Autons/skills.hpp"
#include "main.h"
#include "api.h"
#include "auton.hpp"
#include "config.hpp"
#include "movement.hpp"
#include "robot.hpp"
#include "odom.hpp"
#include "PID.hpp"
#include "initialize.hpp"

  // 4.6, 7,
  //18.6, 8.7
  // 22, -10

void A()
  { //curve turn to grab the red goal
    SPEEDRATIO = 0.35;
    double finalX = 24.3;
    double finalY = -93;
    std::vector<CurvePoint> allPoints;
    CurvePoint start(pos_x, pos_y, 0, 0, 10, 5, 1);
    
    CurvePoint newPoint1(4.6, 7, 0, 0, 10, 5, 1);
    CurvePoint newPoint2(18.6, 8.7, 0, 0, 10, 5, 1);
    CurvePoint newPoint3(24, -10, 0, 0, 10, 5, 1);

    CurvePoint end(finalX, finalY, 0, 0, 10, 5, 1);
    allPoints.push_back(start);
    allPoints.push_back(newPoint1); allPoints.push_back(newPoint2); allPoints.push_back(newPoint3);
    allPoints.push_back(end);

    maxSpeed = 9000;
    while (true)
    {
      if (sqrt(pow(finalX - pos_x, 2) + pow(finalY - pos_y, 2)) < 11)
      {

        arcMovementPID(finalX, finalY, false, true, false);
        break;
      }
      followCurve(allPoints, 0, false);
      liftNoLoop(2500);
      if( 175 < theta*180/M_PI && theta*180/M_PI < 185){
         SPEEDRATIOPID = 0.8; GRABNGO = true;
         arcMovementPID(finalX,finalY,false,true,false,2500);
         GRABNGO =false;
         break;
      }  
    }
  }

  void B(){
    SPEEDRATIO = 0.35;
    double finalX = 98;
    double finalY = -47;
    std::vector<CurvePoint> allPoints;
    CurvePoint start(pos_x, pos_y, 0, 0, 10, 5, 1);
    
    CurvePoint newPoint1(72.5, -47, 0, 0, 10, 5, 1);
    CurvePoint end(finalX, finalY, 0, 0, 10, 5, 1);
    allPoints.push_back(start);
    allPoints.push_back(newPoint1);
    allPoints.push_back(end);

    maxSpeed = 9000;
    while (true)
    {
      if (sqrt(pow(finalX - pos_x, 2) + pow(finalY - pos_y, 2)) < 11)
      {

        arcMovementPID(finalX, finalY, false, true, false);
        break;
      }
      followCurve(allPoints, 0, false);
      liftNoLoop(45000);
      if( 85 < theta*180/M_PI && theta*180/M_PI < 95){
         SPEEDRATIOPID = 0.8; GRABNGO = true;
         arcMovementPID(finalX,finalY,false,true,false,45000);
         GRABNGO =false;
         break;
      }  
    }
  }