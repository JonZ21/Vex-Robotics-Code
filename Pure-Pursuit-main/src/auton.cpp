#include "auton.hpp"
#include "robot.hpp"
#include "movement.hpp"
#include "config.hpp"
#include "main.h"
#include "api.h"
#include "odom.hpp"
#include "PID.hpp"
#include "initialize.hpp"
#include "Autons/mogoWP.hpp"
#include "Autons/skills.hpp"
bool lowStackPos = false;
int maxSpeedLift = 10000;
int lowStackTarget = 2080;
int highStackTarget = 2400;
int clampTarget = 14; // less than 14 mm 
int grabberDistanceValue = 37;
// void park()
// {
//   bool parking = false;
//   int parkingStartTime = 10000000;
//   int localTime = 0;
//   double parkingConstant = 21;
//   while (true)
//   {
//     backLift.move(25);
//     // f(abs(inert.get_pitch()) > 1) {frontLift.move(-30);}

//     if (abs(inert.get_pitch() > 21))
//     {
//       if (!parking)
//         parkingStartTime = localTime;
//       parking = true;
//     }

//     if (abs(inert.get_pitch()) < 15 && !parking)
//     {
//       spinLeft(118);
//       spinRight(118);
//     }

//     else if (parking && localTime - parkingStartTime == 1000)
//       parkingConstant = abs(inert.get_pitch());

//     else if (abs(inert.get_pitch()) < parkingConstant - 0.8 && parking && localTime - parkingStartTime > 1000)
//     {
//       // drive(-100, 2000, 3.5);
//       stop();
//       backLift.move(0);
//       frontLift.move(0);
//       while (true)
//       {
//         changeBrake(S_HOLD, 0, 2.5);
//       }
//     }

//     if (localTime % 50 == 0)
//     {
//       if (parking)
//         con.print(0, 0, "t: %.1f", parkingConstant);
//     }

//     delay(5);
//     localTime += 5;
//   }

  void setClamp(bool value){
    if(value){
      clamp.set_value(0);
    } else{
      clamp.set_value(1);
    }
  }
  void openClamp(){
    clamp.set_value(0);
    CLAMPUP = true;
  }
  void closeClamp(){
    clamp.set_value(1);
    CLAMPUP = false;
  }
  void setTilter(bool value){
    if(value){
      tilter.set_value(0);
      tilter2.set_value(1);
    } else{
      tilter.set_value(1);
      tilter2.set_value(0);
    }
  }

  void softTiltDrop(){
    tilter.set_value(0);
    tilter2.set_value(0);
  }
  
  bool GRABBED = false;
  void setGrab(bool value){
    if(value){
      grabber.set_value(0);
      GRABBED = true;
    } else{
      grabber.set_value(1);
      GRABBED = false;
    }
  }
  
  void grab(){
    grabber.set_value(1);
    GRABBED = true;
  }
  void unGrab(){
    grabber.set_value(0);
    GRABBED = false;
  }


  void runIntake()
  {
    intake.move_voltage(12000);
  }
  void stopIntake()
  {
    intake.move_velocity(0);
  }

  bool LIFTMIDDLE = false;
  void liftTo(int target) // auto detects if there's a goal or not
  {
    int current = liftPot.get_position();

    double kP = 2;
    if (grabbed)
    {
      std::cout<<"GRABBED \n";
      kP = 3;
    }
    if (LIFTMIDDLE)
    {
      kP = 3.6;
    }
    double error;
    double speed;
    int counter = 0;
    int kD;
    double derivative = 10000, prevError = 0;
    int thresholdTimer = 0;
    int failsafeCounter = 0;
    std::cout<<"Lift to: " << target<< std::endl;
    while (true)
    {
      current = liftPot.get_position();
      error = target - current;
      speed = error * kP;
      if (speed > maxSpeedLift)
        speed = maxSpeedLift;
      else if (speed < -maxSpeedLift)
        speed = -maxSpeedLift;

      if (abs(error) < 500)
      {
        thresholdTimer++;
      }
      else
      {
        thresholdTimer = 0;
      }

      if (thresholdTimer > 30)
      {
        std::cout<<"reached goal";
        lift.move_velocity(0);
        break;
      }

      lift.move_voltage(-speed);

      prevError = error;
      derivative = error - prevError;
      std::cout << "Speed:" << speed << " error:" << error << std::endl;

      if (failsafeCounter % 150 == 0 && failsafeCounter != 0)
      {
        if (fabs(prevError - error) < 75)
        {
          std::cout << "lift broke" << std::endl;
          lift.move_velocity(0);
          break;
        }

        prevError = error;
      }
      failsafeCounter++;
      pros::delay(10);
    }
  }

   void liftNoLoop(int target) 
  {
    int current = liftPot.get_position();
    double kP = 2;
    if (grabbed)
    {
      kP = 3;
    }
    double error;
    double speed;
    error = target - current;
    speed = error * kP;
    lift.move_voltage(-speed);
    std::cout << "Speed:" << speed << std::endl;

    if(fabs(error) < 100){
       lift.move_velocity(0);
    }

    // std::cout<<"speed: " << speed<< std::endl;
  }
  void liftUp(int voltage)
  {
    lift.move_voltage(-voltage);
  }
  void liftDown()
  {
    lift.move_voltage(10000);
  }
  void liftStop()
  {
    lift.move_voltage(0);

  }

  void setLiftBrake()
  {
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  }
  void setLiftHold()
  {
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  }

  void frontGrabberAuton()
  {
    if (grabberDistance.get() < grabberDistanceValue)
    {
      setGrab(true);
    }
  }

  bool grabbed()
  {
    if (grabberDistance.get() < grabberDistanceValue)
      return true;
    return false;
  }

  void clampAuton()
  {
    if (clampDistance.get() < clampTarget && clampDistance.get() != 0)
    {
      closeClamp();
    }
  }


  bool clamped()
  {
    if (clampDistance.get() < clampTarget && clampDistance.get() != 0)
      return true;
    return false;
  }

  void route1()
  {
    bool backwards = false;
    double finalX = 30.01;
    double finalY = 0.4;
    std::vector<CurvePoint> allPoints;
    CurvePoint start(0, 0, 0, 0, 13, 5, 1);
    CurvePoint newPoint2(0.1, 50.0, 0, 0, 13, 5, 1);
    CurvePoint newPoint1(30, 50.1, 0, 0, 13, 5, 1);
    CurvePoint newPoint4(30.21, 0.5, 0, 0, 13, 5, 1);
    // x = -51.5535 y = 51.4888
    CurvePoint newPoint3(finalX, finalY, 0, 0, 13, 5, 1);

    allPoints.push_back(start);
    allPoints.push_back(newPoint2);
    allPoints.push_back(newPoint1);
    allPoints.push_back(newPoint4);
    allPoints.push_back(newPoint3);
    int counter = 0;
    while (true)
    {
      counter++;
      if (sqrt(pow(finalX - pos_x, 2) + pow(finalY - pos_y, 2)) < 4)
      {
        straightPID(finalX, finalY, 0.05);
        break;
      }
      followCurve(allPoints, 0, backwards);
    }
    //  updateOdom.remove(); // shutsdown the odom thread.
  }

  void route2()
  {
    bool backwards = true;
    double finalX = -30.2;
    double finalY = -0.4;
    std::vector<CurvePoint> allPoints;
    CurvePoint start(0, 0, 0, 0, 10, 5, 1);
    CurvePoint newPoint2(-0.1, -50.0, 0, 0, 10, 5, 1);
    CurvePoint newPoint1(-30, -50.1, 0, 0, 10, 5, 1);
    CurvePoint newPoint4(-30.21, -0.5, 0, 0, 10, 5, 1);

    CurvePoint newPoint3(finalX, finalY, 0, 0, 10, 5, 1);

    allPoints.push_back(start);
    allPoints.push_back(newPoint2);
    allPoints.push_back(newPoint1);
    allPoints.push_back(newPoint4);
    allPoints.push_back(newPoint3);

    while (true)
    {
      if (sqrt(pow(finalX - pos_x, 2) + pow(finalY - pos_y, 2)) < 5)
      {
        arcMovementPID(finalX, finalY, backwards, false, false);
        break;
      }
      followCurve(allPoints, 0, backwards);
    }
    //  updateOdom.remove(); // shutsdown the odom thread.
  }
  void route3()
  {
    bool backwards = false;
    double finalX = 55;
    double finalY = 29;
    std::vector<CurvePoint> allPoints;
    CurvePoint start(pos_x, pos_y, 0, 0, 10, 5, 1);
    CurvePoint newPoint2(2, 29, 0, 0, 10, 5, 1);
    CurvePoint newPoint3(finalX, finalY, 0, 0, 10, 5, 1);

    allPoints.push_back(start);
    allPoints.push_back(newPoint2);
    allPoints.push_back(newPoint3);

    while (true)
    {
      if (sqrt(pow(finalX - pos_x, 2) + pow(finalY - pos_y, 2)) < 11)
      {
        arcMovementPID(finalX, finalY, backwards, false, false);
        break;
      }
      followCurve(allPoints, 0, backwards);
    }
    //  updateOdom.remove(); // shutsdown the odom thread.
  }
  void route_afterFirstGoalGetHomeRowWithBack(double x, double y)
  {
    bool backwards = true;
    double finalX = 30.2;
    double finalY = 0.4;
    std::vector<CurvePoint> allPoints;
    CurvePoint start(x, y, 0, 0, 10, 5, 1);
    CurvePoint newPoint2(0.1, 50.0, 0, 0, 10, 5, 1);
    CurvePoint newPoint3(finalX, finalY, 0, 0, 10, 5, 1);

    allPoints.push_back(start);
    allPoints.push_back(newPoint2);

    while (true)
    {
      if (sqrt(pow(finalX - pos_x, 2) + pow(finalY - pos_y, 2)) < 5)
      {
        arcMovementPID(finalX, finalY, backwards, false, false);
        break;
      }
      followCurve(allPoints, 0, backwards);
    }
  }
  void extraComponents()
  {
    frontGrabberAuton();
  }

  void route4()
  {

    double finalX = -37.1;
    double finalY = 10;
    std::vector<CurvePoint> allPoints;
    CurvePoint start(pos_x, pos_y, 0, 0, 14, 5, 1);
    CurvePoint newPoint2(-37, 80, 0, 0, 14, 5, 1);
    CurvePoint newPoint7(finalX, finalY, 0, 0, 14, 5, 1);
    allPoints.push_back(start);
    allPoints.push_back(newPoint2);
    allPoints.push_back(newPoint7);

    while (true)
    {
      if (pos_y > 54)
      {
        maxSpeed = 12000;
      }
      else
      {
        maxSpeed = 6000;
      }
      if (sqrt(pow(finalX - pos_x, 2) + pow(finalY - pos_y, 2)) < 15)
      {
        maxSpeedPID = 6000;
        arcMovementPID(finalX, finalY, false, false, false);
        stop();
        break;
      }
      followCurve(allPoints, 0, false);
    }
  }
  int timeSec = 0;

  void time(void *param)
  {
    if (timeSec < 20000)
    {
      while (true)
      {
        if (timeSec > 14800 && autonChoice != 9 && timeSec < 15800) // && Autonchoice != 9
        {
          // sixMotorDrive = true;
          // set6MDrive();
          tiltDown();
          // openClamp();
          stop();
          // std::cout<<"TIME: " << timeSec;
        }
        mutex.take(1);
        // std::cout<<"time: " << timeSec<<std::endl;
      
        timeSec++;
        mutex.give();
        pros::delay(1);
      }
    }
  }

  void SR1()
  { // SKills route 1
    double finalX = 50;
    double finalY = -4.3;
    std::vector<CurvePoint> allPoints;
    CurvePoint start(pos_x, pos_y, 0, 0, 10, 5, 1);
    CurvePoint newPoint2(13, 9, 0, 0, 10, 5, 1);
    CurvePoint newPoint7(finalX, finalY, 0, 0, 10, 5, 1);
    allPoints.push_back(start);
    allPoints.push_back(newPoint2);
    allPoints.push_back(newPoint7);

    while (true)
    {
      if (sqrt(pow(finalX - pos_x, 2) + pow(finalY - pos_y, 2)) < 11)
      {
        arcMovementPID(finalX, finalY, false, false, false);
        stop();
        grabber.set_value(1);
        stop();
        break;
      }
      liftNoLoop(RESTING_LIFT_HEIGHT);

      followCurve(allPoints, 0, false);
    }
  }
  void SR2()
  { // grabbing r1 after platforming b1
    double finalX = 96.67;
    double finalY = 3;
    std::vector<CurvePoint> allPoints;
    CurvePoint start(pos_x, pos_y, 0, 0, 10, 5, 1);
    CurvePoint newPoint2(90, -36, 0, 0, 10, 5, 1);
    CurvePoint newPoint7(finalX, finalY, 0, 0, 10, 5, 1);
    allPoints.push_back(start);
    allPoints.push_back(newPoint2);
    allPoints.push_back(newPoint7);

    while (true)
    {
      if (sqrt(pow(finalX - pos_x, 2) + pow(finalY - pos_y, 2)) < 11)
      {

        arcMovementPID(finalX, finalY, true, false, true);
        clamp.set_value(0);
        stop();
        liftStop();

        break;
      }

      followCurve(allPoints, 0, true);
    }
  }


  void plusRoute()
  { // grabbing r1 after platforming b1
    double finalX = -69.09;
    double finalY = 32.1;
    std::vector<CurvePoint> allPoints;
    CurvePoint start(pos_x, pos_y, 0, 0, 10, 5, 1);
    CurvePoint newPoint2(-18.5, 32, 0, 0, 10, 5, 1);
    CurvePoint newPoint7(finalX, finalY, 0, 0, 10, 5, 1);
    allPoints.push_back(start);
    allPoints.push_back(newPoint2);
    allPoints.push_back(newPoint7);

    maxSpeed = 12000;
    while (true)
    {
      if (sqrt(pow(finalX - pos_x, 2) + pow(finalY - pos_y, 2)) < 11)
      {

        arcMovementPID(finalX, finalY, false, false, false);
        // clamp.set_value(0);
        // stop();
        // liftStop();


        break;
      }
      if(pos_x < -17){
          maxSpeed = 5000;
      } else{
        maxSpeed = 12000;
      }
      // leftLift.move_voltage(2000);
      // rightLift.move_voltage(-2000);

      followCurve(allPoints, 0, false);
    }
  }

  
  void park(){
    bool parkToggle = false;
    int count = 0;
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    while(true){

      if(imuRight.get_roll() > 15){
        count++;
      }
      
      if(count < 35){
         left_move(-12000);
        right_move(12000);
      } else{
        left_move(-6500);
        right_move(6500);
      }

      if(imuRight.get_roll() > 21){
        parkToggle = true;
      }
      if(parkToggle && imuRight.get_roll() < 20.3){
        left_move(8000);
        
        right_move(-8000);
         tiltDown();

        pros::delay(140);
        setBreakTypeHold();
        stop();
        break;
      }
      pros::delay(10);
    }
  }


  //5.8, 13 - 33.8,13 - 36, -2.6, -26, -50


  void WP1()
  { // grabbing r1 after platforming b1
    double finalX = 35.59;
    double finalY = -60.01; // special
    std::vector<CurvePoint> allPoints;
    CurvePoint start(pos_x, pos_y, 0, 0, 10, 5, 1);
    CurvePoint newPoint1(5.8, 13, 0, 0, 10, 5, 1);
    CurvePoint newPoint2(33.8, 13, 0, 0, 10, 5, 1);
    CurvePoint newPoint3(35.6, -2.6, 0, 0, 10, 5, 1);
    CurvePoint newPoint7(finalX, finalY, 0, 0, 10, 5, 1);
    allPoints.push_back(start);
    allPoints.push_back(newPoint2);
    allPoints.push_back(newPoint7);

    maxSpeed = 9000;
    while (true)
    {
      if (sqrt(pow(finalX - pos_x, 2) + pow(finalY - pos_y, 2)) < 11)
      {

        arcMovementPID(finalX, finalY, false, false, false);
        // clamp.set_value(0);
        // stop();
        // liftStop();


        break;
      }
      if(pos_y < -60){ // to prevent absolute chaos
        break;
      }
      liftNoLoop(20000);
      if(pos_y < -2){
        maxSpeed = 3400;
        intakeSpeed = 12000;
      } else{
        maxSpeed = 10000;
      }

      if( 175 < theta*180/M_PI && theta*180/M_PI < 185){
         SPEEDRATIOPID = 0.8;
         maxSpeedPID = 4000;
         arcMovementPID(finalX,finalY,false,false,false,20001);
         GRABNGO =false;
         break;
      }  
      // leftLift.move_voltage(2000);
      // rightLift.move_voltage(-2000);

      followCurve(allPoints, 0, false);
    }
  }
  void RS1()
  { // grabbing r1 after platforming b1
    double finalX = -30;
    double finalY = 31.7;
    std::vector<CurvePoint> allPoints;
    CurvePoint start(pos_x, pos_y, 0, 0, 10, 5, 1);
    CurvePoint newPoint3(-26, 34, 0, 0, 10, 5, 1);
    CurvePoint newPoint7(finalX, finalY, 0, 0, 10, 5, 1);
    allPoints.push_back(start);
    allPoints.push_back(newPoint3);
    allPoints.push_back(newPoint7);

    maxSpeed = 12000;
    while (true)
    {
      if (sqrt(pow(finalX - pos_x, 2) + pow(finalY - pos_y, 2)) < 11)
      {

        arcMovementPID(finalX, finalY, false, false, false);
        // clamp.set_value(0);
        // stop();
        // liftStop();


        break;
      }
      liftNoLoop(20000);

      if(pos_x < 0){
        maxSpeed = 4000;
      }

      if( 175 < theta*180/M_PI && theta*180/M_PI < 185 && pos_x < -20){
         SPEEDRATIOPID = 0.8;
         maxSpeedPID = 4000; DDSTRAIGHTMAX = 4000;
         arcMovementPID(finalX,finalY,false,false,false,20000);
         GRABNGO =false;
         break;
      }  
      followCurve(allPoints, 0, false);
    }
  }
  //98 15, 
  void A5()
  { // grabbing r1 after platforming b1
    double finalX = 8;
    double finalY = 30;
    SPEEDRATIOPID = SPEEDRATIOPID - 0.05;
    std::vector<CurvePoint> allPoints;
    CurvePoint start(pos_x, pos_y, 0, 0, 14, 5, 1);
    CurvePoint newPoint3(50, 29.5, 0, 0, 10, 5, 1);
    CurvePoint newPoint7(finalX, finalY, 0, 0, 10, 5, 1);
    allPoints.push_back(start);
    allPoints.push_back(newPoint3);
    allPoints.push_back(newPoint7);

    maxSpeed = 10000;
    while (true)
    {
      if (sqrt(pow(finalX - pos_x, 2) + pow(finalY - pos_y, 2)) < 11)
      {

        arcMovementPID(finalX, finalY, false, false, false);
        // clamp.set_value(0);
        // stop();
        // liftStop();  


        break;
      }
      liftNoLoop(20000);

      if(pos_x < 53){
        maxSpeed = 3000;
      }

      if((355 < theta*180/M_PI || theta*180/M_PI < 5)  && pos_x < 55){
        SPEEDRATIO = 0.8;
      }

      if(( 355 < theta*180/M_PI || theta*180/M_PI < 5)  && pos_x < 20){
         SPEEDRATIOPID = 0.8;
         maxSpeedPID = 3000; DDSTRAIGHTMAX = 3000;
         arcMovementPID(finalX,finalY,false,false,false,20000);
         GRABNGO =false;
         break;
      }  
      followCurve(allPoints, 0, false);
    }
  }
  int AUTO_ID = 0;
  void autonomous()
  {
    pros::Task updateOdom(odomTask, NULL, TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT, "Odom Update"); // while loop
    pros::Task updateTime(time, NULL, TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT, "Time Update");
    std::cout << "AUTON START" << std::endl;
    timeSec = 0;
    TURNPIDKP = 380;
    STRAIGHTPIDKP = 1150;

    if(autonChoice == 1){ // WP 
      closeClamp();
      setGrab(false);
      pros::delay(200);
      tiltUp();
      intakeSpeed = 12000;
      WP1();
      
      // turnPID(135,1.3);
      slowTiltDown();   
      pros::delay(500);
      tiltDown();openClamp();
      // ddStraight(10,false); maxSpeedPID = 8000;
      maxSpeedPID = 9000;

      // turnToPoint(23.01,-70,false,40000); // turn so no counter

      // AUTO_ID = 99; // basically set timer for this to be 0 so it doesnt wait
      // arcMovementPID(23,-70.01,false,false,true,40000); // SPECIAL CASE NO KIZZY GOOD

      // turnToPoint(23,-95,true,40000);
      AUTO_ID = 99;
      arcMovementPID(23,-98.001,true,false,true,40000); // this backup, if detect no movement for less time then go
      closeClamp();
      AUTO_ID = 0;
      pros::delay(100);
      tiltUp();
      maxSpeedPID = 9000;
      ddStraight(10,false);

      maxSpeedPID = 5000;
      // turnToPoint(55,-85,false,20000);
      arcMovementPID(55.01,-85,false,false,false,20000); // if past 55 then break
       
      maxSpeedPID = 9000; DDSTRAIGHTMAX = 8000;
      while(true){
        left_move(12000); // -12000
        right_move(-12000);


       if(pos_x < 43){
         stop();
         break;
       }
      }
      
      slowTiltDown();
      pros::delay(500);
      tiltDown();
      liftTo(2500);



    } else if(autonChoice == 2){ // LS goal rush  no wp
      eClaw.set_value(1);   
      eRelease.set_value(0);
      setGrab(false);

      while(pos_y < 26){
        left_move(-12000); // -12000
        right_move(12000);
        pros::delay(10);
      }
      int eCounter = 0;
      int bruhCounter = 0;
      while(pos_y > 6.5){
        if(eCounter > 1){
         eRelease.set_value(1);
        }
        left_move(9000);
        right_move(-12000);
        eCounter++;
        pros::delay(10);
      
      }
      
      while(true){
        if( pos_y > 7){
          left_move(9000);
          right_move(-12000);
          pros::delay(10);
        } else{
          stop();
        }
      }
      // liftTo(50000);
      // turnToPoint(-15,0.5,true,50000);
      // arcMovementPID(-15,0.5,true,false,true,50000);
      // turnToPoint(0,0.5,true,50000);
      // arcMovementPID(0,0.5,true,false,true,50000);
      // closeClamp();
      // tiltUp();
      // pros::delay(500);
      // ddStraight(10,false);
      // pros::delay(100);
      // tiltDown();
      // liftTo(3000);


            // liftTo(8000); setLiftHold(); liftStop();
      // turnPID(270,1.3);

      // arcMovementPID(0,0,true,false,true);
      // closeClamp();
      // pros::delay(200);
      // tiltUp();
      // pros::delay(200);
      // RS1();
      // while(true){
      //   liftNoLoop(8000);
      // }


      // 20, 22


      // pros::delay(250);
      // arcMovementPID(25,20,true,false,true);
      // pros::delay(100);
      // closeClamp();
      // pros::delay(200);
      // tiltUp();

    
    }
    else if( autonChoice == 3) // RS 
{
      eClaw.set_value(1);   
      eRelease.set_value(0);
      setGrab(false);

      while(pos_y < 26){
        left_move(-12000); // -12000
        right_move(12000);
        pros::delay(10);
      }
      int eCounter = 0;
      int bruhCounter = 0;
      while(pos_y > 15.5){
        if(eCounter > 1){
         eRelease.set_value(1);
        }
        left_move(9000);
        right_move(-12000);
         if(pos_y < 20)
        {
          bruhCounter++;
          std::cout<<"bruh counter " << bruhCounter<< std::endl;
          if(bruhCounter > 20 && bruhCounter <  55){
            eClaw.set_value(1);
          } else{
           eClaw.set_value(0);

          }
        }
        eCounter++;
        pros::delay(10);
      }
      eClaw.set_value(1);
      stop();  pros::delay(700); setGrab(false);     
      eClaw.set_value(0);
      pros::delay(500); eClaw.set_value(1); 
      pros::delay(500);
      eClaw.set_value(0);

      turnPID(0,1);
      maxSpeedPID = 5000;
      DDSTRAIGHTMAX = 5000; GRABNGO = true;
      turnToPoint(3,18,false);
      arcMovementPID(3,15.5,false,true,false,3100); GRABNGO = false;
      setGrab(1);
      liftTo(20000); liftStop();
      turnPID(270,1.3);
      arcMovementPID(30,20.6,true,false,true,20000);
      closeClamp();
      pros::delay(200);
      tiltUp();
      pros::delay(500);
      liftTo(20000);
      arcMovementPID(-1,19,false,false,false,20000);
      pros::delay(100);
      tiltDown();
      liftTo(3000);


}
else if( autonChoice == 4){ // left side goal match load
      pros::delay(500);
      closeClamp();
      setGrab(false);
      pros::delay(200);
      tiltUp(); pros::delay(500);
      maxSpeedPID = 4000;
      DDSTRAIGHTMAX = 4000;
      liftUp(50000); setLiftHold();
      pros::delay(500);
      while(pos_y < 15){
        left_move(-4000); // -12000
        right_move(4000);
        liftNoLoop(50000);
      }
         stop();
      pros::delay(500);

      while(pos_y > 5){
        left_move(4000); // -12000
        right_move(-4000);
        liftNoLoop(50000);
     

      }
         stop();
       pros::delay(500);
      while(pos_y < 15){
        left_move(-4000); // -12000
        right_move(4000);
        liftNoLoop(50000);
      }
         stop();
      pros::delay(500);
      while(pos_y > 5){
        left_move(4000); // -12000
        right_move(-4000);
        liftNoLoop(50000);
      

      }
     while(pos_y < 15){
        left_move(-4000); // -12000
        right_move(4000);
        liftNoLoop(50000);


      }   stop();
      pros::delay(500);
      while(pos_y > 13){
        left_move(4000); // -12000
        right_move(-4000);
        liftNoLoop(50000);


      }
      stop();
      slowTiltDown(); 
      pros::delay(500);
      tiltDown();
      liftTo(3000);

} else if (autonChoice == 5){ // new erctor left side win point
    eClaw.set_value(1);   //flipout and erect
    eRelease.set_value(0);
    tiltDown();
    setGrab(false);

    while(pos_y < 19){
      left_move(-12000); // -12000
      right_move(12000);
      pros::delay(10);
    }
    eRelease.set_value(1);
    // setBreakTypeHold();
    // stop();
    // pros::delay(5000);

    left_move(12000); // backup
    right_move(-12000);
    bool toggleTurn = false;
    while(true){
      // stop();
      if(pos_y > 14){
        left_move(12000); // backup
        right_move(-12000);
      } else if (pos_y < 14){
        toggleTurn = true;
      }

      if(toggleTurn){
        left_move(-12000); // turn
        right_move(-12000);
        if(theta*180/M_PI > 10){
          left_move(-8000); // -12000
          right_move(8000);
        }
      }
        // left_move(-8000); // backup
        // right_move(0);
        if (grabberDistance.get() < grabberDistanceValue && grabberDistance.get() != 0){
          stop();
          setGrab(true);
          pros::delay(100);
          eClaw.set_value(0);
          break;
        }
    }
    liftTo(20000);
    // A5();
    maxSpeedTurn = 5000;
      
    maxSpeedPID = 8000;
    turnToPoint(98,15,true);
    SPEEDRATIOPID = 0.8;
    arcMovementPID(98,15,true,false,true,25000);
    closeClamp(); pros::delay(200); tiltUp();
    // A5();

    // turnToPoint(50,26,false,25000);
    arcMovementPID(50,26,false,false,false,25000);
    maxSpeedPID = 5000;
    arcMovementPID(8,26,false,false,25000);

    

    

} else if (autonChoice == 6){
  wpRing.set_value(1);
  eClaw.set_value(1);   //flipout and erect
  eRelease.set_value(0);
  tiltDown();
  setGrab(false);
  ddStraightNoFail(24.5,false);
  wpRing.set_value(0);
  eRelease.set_value(1);


  ddBackwardsNoFail(20,false);

bool toggleTurn = false;
    while(true){
      // stop();
      if(pos_y > 14 && !toggleTurn){
        left_move(12000); // backup
        right_move(-12000);
      } else if (pos_y < 14){
        toggleTurn = true;
      }

      if(toggleTurn){
          left_move(-12000); // -12000
          right_move(4000);
        }
        // left_move(-8000); // backup
        // right_move(0);
        if (grabberDistance.get() < grabberDistanceValue && grabberDistance.get() != 0){
          stop();
          setGrab(true);
          pros::delay(100);
          eClaw.set_value(0);
          break;
        }
    }
  
    liftTo(10000); setLiftHold(); liftStop();
    // A5();
    while(pos_y > 15){
      left_move(8000); // backup
      right_move(8000);
    }
    stop();
    maxSpeedTurn = 5000;
      
    maxSpeedPID = 8000;
    turnToPoint(98,15,true,10000);
    arcMovementPID(98,15,true,false,true,10000);
    closeClamp(); pros::delay(200); tiltUp();
    // A5();

    turnToPoint(55,26,false,20000);
    arcMovementPID(55,26,false,false,false,20000);
    maxSpeedPID = 5000;
    turnToPoint(8,26,false,20000);
    arcMovementPID(8,26,false,false,20000);

  // tiltDown();
  // pros::delay(200);
  // closeClamp(); 
  // pros::delay(200);
  // tiltUp();
  // liftTo(25000);
  // A5();
}
    if(autonChoice == 9){ // skills
       //lif1 = above ground so you dont drag 

      // closeClamp();
      // tiltUp();
      // setGrab(true);
      // turnPID(0,2);
      // ddStraight(8,false);
      // liftTo(500);
      // park();

      // pros::delay(1000000);
      
      // liftTo(60000);
      // closeClamp(); tiltUp();
      // while(true){
      //   liftNoLoop(60000);
      //   pros::delay(10);
      // }

      // setGrab(true);
      // closeClamp(); tiltUp();
      // liftTo(50000); DDSTRAIGHTMAX = 6000;
      // ddStraight(7,false);
      // liftTo(500);
      // park();
      // pros::delay(100000000);


      //b1
      DDSTRAIGHTMAX = 9000;
      maxSpeedTurn = 8000;
      closeClamp();
      setGrab(false);
      pros::delay(200);
      tiltUp();
      A(); SPEEDRATIOPID = 0.7; // grab the second blue
      
      setGrab(true);liftUp(12000); setLiftHold(); 
      //lift1
      //intake
      arcMovementPID(22,-76,true,false,false,20000); liftStop();// backup s
      ARCTURNEXECPTION = true;
      turnPID(64,1.3); // turn to face b platform
      ARCTURNEXECPTION = false;
   
      AUTO_ID = 1;
      arcMovementPID(95,-38,false,false,false,69000); // drive inbetween neutrals to platform // 80 , -38 // 97, -28
      maxSpeedPID = 9000;
      AUTO_ID = 0;

      turnPID(85,1); // turn and platform
      liftUp(-4000);
      ddStraight(4,false);
      liftTo(33000);
      pros::delay(200);
      setGrab(false);
      liftUp(3000);
      ddBackwards(6,false); GRABNGO = true;
      arcMovementPID(71,-36.3,false,true,false,2600); // grab mid 
      GRABNGO = false;
      setLiftHold(); liftStop(); setGrab(true); liftUp(12000); maxSpeedTurn = 5000; maxSpeedPID = 7000;
      arcMovementPID(98,-47,false,false,false,45000); maxSpeedTurn = 9000;  maxSpeedPID = 9000;// platform middle 
      liftStop(); turnPID(88,1);
      ddStraight(3,false);
      liftTo(33000); // lowest ou can go on the platform
      maxSpeedLift = 10000;
      pros::delay(200);
      setGrab(false);
      pros::delay(200);
      left_move(3000);
      right_move(-3000);
      pros::delay(300);
      liftUp(5000);
      pros::delay(100); // backup while lifting up
      ddBackwards(6,false); // backup after middle
      liftUp(-8000);

      GRABNGO = true;
      arcMovementPID(62.5,-7.4,false,true,false,2600); GRABNGO = false; // grab y2
      setGrab(true); 
    
      // if(!grabbed()){pros::delay(200);}

      if(grabbed()){
        arcMovementPID(97,-19,false,false,false,47000);
        liftTo(36000);
        setGrab(false);
        ddBackwards(4,false);
      }
      else{
        arcMovementPID(93,-19,false,false,false,47000); 
        setGrab(false); 

      }
    
      turnPID(180,1); liftUp(-5000);
      slowTiltDown();
      pros::delay(500); openClamp(); tiltDown();
      arcMovementPID(pos_x,pos_y- 10,false,false,false,3200); GRABNGO = true;
      ARCTURNEXECPTION = true;
      for(int i = 0; i < 20; i++){
        right_move(-8000);
        left_move(-8000);
        liftUp(-8000);
        pros::delay(10);
      }
      ARCTURNEXECPTION = false;
      maxSpeedPID = 7000; DDSTRAIGHTMAX = 7000;
      arcMovementPID(pos_x,pos_y + 13,false,true,false,3200); GRABNGO = false;
      setGrab(true); liftUp(12000); 
      liftTo(55000); liftStop(); maxSpeedTurn = 9000;  maxSpeedPID = 9000; //plat b2 
      turnPID(126.4,1.5);
      DDSTRAIGHTMAX = 7000; // please just work
      ddStraight(17,false); DDSTRAIGHTMAX = 9000;
      setGrab(false);
      ddBackwards(8,false);
      turnToPoint(97,17,true,2600);
      arcMovementPID(97,18.5,true,false,true,2600); liftStop();
      closeClamp(); pros::delay(200);
      tiltUp();
     
      AUTO_ID = 2;
      arcMovementPID(67.4,-62.5,false,true,false,2600);
      AUTO_ID = 0;
      
      setGrab(true);
      turnToPoint(100,-54.5,false,50000);
      arcMovementPID(100,-54.5,false,false,false,50000);
      setGrab(false); //platform y3
      pros::delay(100);
      ddBackwards(6,false); // after 5
      arcMovementPID(97.9,-87,false,false,false,2600);

      turnToPoint(111,-73,false,2600); maxSpeedPID = 7000; GRABNGO = true;
      arcMovementPID(111,-73,false,true,false,2600); GRABNGO = false;
      setGrab(1);

      maxSpeedPID = 10000; DDSTRAIGHTMAX = 12000;
      ddBackwards(18,false);
      turnToPoint(-100,pos_y,false,50000); liftStop();
      arcMovementPID(0.3,pos_y,false,false,false,50000);
      // ARCTURNEXECPTION = true;
      ddBackwards(2,false);
      turnPID(2.4,2.5);
      ddStraight(4,false);
      liftTo(500);
      park();

      // 97.5, -55.9


      //40000
      
    
//                         }

      //88, -35, 
      // 


      // //platform b1


      // //b2
      // arcMovementPID(90,-35,true,false,false); // backup
      // turnPID(180,1); // turn and drop b2
      // double ref = pos_x;
      // arcMovementPID(pos_x,-44,false,false,false); // drive fwd
      // turnPID(0,1);// turn to face b2 
      // arcMovementPID(ref,-40,false,true,false); // pickup b2
      // turnPID(90,1);
      // //platform b2

      // //r1 pickup
      // arcMovementPID(97.8,12,true,false,true);
      // closeClamp();
      // pros::delay(200);
      // tiltUp();


      // //y1
      // arcMovementPID(69,-1.6,false,true,false);
      // setGrab(true);

      
      
      
      
      // setBreakTypeCoast();



    }

    if (autonChoice < 9)
    { // matchAutons
      SPEEDRATIO = 0.25;
      SPEEDRATIOPID = 0.7;
    }

//     if (autonChoice == 1) // Right Side middle goal and alliance rings
//     {
//       SPEEDRATIOPID = 0.6;
//       mogoWP_auton();
//       stop();
//     }
//     else if (autonChoice == 2) // Right side side GOAL and alliance rings
//     {
//       setGrab(false);
//       goalCover.set_value(1); GOALCOVERDOWN = true;
//       while (true)
//       { // grab first goal
//         liftNoLoop(390);
//         left_move(-12000);
//         right_move(12000);

//         if (grabberDistance.get() < grabberDistanceValue)
//         { // if sees goal. clamp and leave
//           setGrab(true);
//           left_move(12000);
//           right_move(-12000);
//           break;
//         }
//         if (pos_y > 50)
//         { // if we are beyond the zone, go back so we don't run into the opposing team
//           setGrab(true);
//           break;
//         }

//         pros::delay(10);
//       }
//       liftStop();
//       setLiftHold();
//       FAILSAFEEXCEPTION = true;
//       maxSpeedPID = 12000;
//       arcMovementPID(pos_x, 19, true, false, false); // backup
//       FAILSAFEEXCEPTION = false;
//       maxSpeedPID = 9000;
//       liftTo(700);
//       setLiftHold();
//       liftStop();
//       turnPID(265, 1.3);
//       arcMovementPID(18, 19, true, false, true); // backup // need to make a break once clamped
//       closeClamp();
//       stop();
//       pros::delay(200);
//       tiltUp();
//       // liftTo(1000);
//       setLiftHold();
//       liftStop();
//       ddStraight(10,false);
//       set4MDrive();
//       // liftTo(700); liftStop(); setLiftHold();
//       pros::delay(200);
//       runIntake();
//       // pros::delay(200);
//       // maxSpeedPID = 4000;
//       turnPID(280,1);
//       plusRoute();
//       // arcMovementPID(14, 55, false, false, false); // make so cgrab and clamp :(
//       // maxSpeedPID = 10000;
//       // arcMovementPID(14, 10, true, false, false);
//       // stopIntake(); // make so cgrab and clamp :(
//       // driveTrans.set_value(1);
//       // intakeTrans.set_value(0);
//       stop();  stopIntake(); goalCover.set_value(0);
//       slowTiltDown(); pros::delay(200);
//       // openClamp();
//       set6MDrive();


//     }
//     else if (autonChoice == 3) // Leftside first goal and alliance rings
//     {
//       // ddBackwards(-6000*30);
//       // turnPID(90,1.3);
//       // pros::delay(1000000);
//       setGrab(false);
//       clamp.set_value(1);
//       goalCover.set_value(1);

//       while (true)
//       { // grab first goal
      
//         left_move(-12000);

//         right_move(10000);

//         if (grabberDistance.get() < grabberDistanceValue)
//         { // if sees goal. clamp and leave
//           grabber.set_value(1);
//           left_move(12000);
//           right_move(-12000);
//           break;
//         }
//         if (pos_y > 50)
//         { // if we are beyond the zone, go back so we don't run into the opposing team
//           grabber.set_value(1);
//           break;
//         }

//         pros::delay(10);
//       }
//       liftStop();
//       FAILSAFEEXCEPTION = true;
//       arcMovementPID(1.8, 13, true, false, false); // backup
//       FAILSAFEEXCEPTION = false;
//       stop();
//       for (int i = 0; i < 30; i++)
//       {
//         liftUp(10000);
//         pros::delay(10);
//       }
//       setLiftHold();
//       liftStop();
//       clamp.set_value(1);
//       turnPID(335, 2);
//       arcMovementPID(9, 2, true, false, true); // backup
//       clamp.set_value(0);
//       pros::delay(100);
//       // clampUp = !clampUp;
//       // driveTrans.set_value(0);
//       // intakeTrans.set_value(1);
//       sixMotorDrive = false;
//       runIntake();
//       for (int i = 0; i < 20; i++)
//       {
//         left_move(-6000);
//         right_move(6000);
//         pros::delay(10);
//       }
//       // arcMovementPID(-2,11, false, false, true); // go straight
//       turnPID(0, 3);
//       runIntake();
//       for (int i = 0; i < 20; i++)
//       {
//         left_move(-10000);
//         right_move(6000);
//         pros::delay(10);
//       }

//       // maxSpeed = 12000;
//       route3();
//       runIntake();
//       // maxSpeed = 9000;
//       runIntake();
//       arcMovementPID(0, 10, true, false, false);
//       stop();
//       stopIntake();
//       clamp.set_value(1);
//       stop();
//       stopIntake();
//     }
//     else if (autonChoice == 4) // leftSide Winpoint
//     {
//       int count = 0; // ring mech
//       setGrab(false); // open grabber
//       while (true)
//       {
//         count++;
//         if (count > 50)
//           break;
//         pros::delay(10);
//       }
//       count = 0;
//       turnPID(90, 1.3);
//       ddBackwards(20,false);
//       turnPID(180,1);
//       openClamp();
//       arcMovementPID(-21, 105, true, false, true);
    
//       closeClamp();
//       left_move(0);
//       right_move(0);
//       pros::delay(200);
//       tiltUp();
      
//       setLiftHold();
//       liftStop();
//       // clampUp = !clampUp
//       set4MDrive();
//       liftTo(534); liftStop(); setLiftHold();
//       pros::delay(200);
//       runIntake();
//       pros::delay(200);
//       ddStraight(24,false); 
//       stopIntake();

//       set6MDrive();
//       pros::delay(200);
//       turnPID(230, 2);
//       liftTo(390);
//       arcMovementPID(-46.8,54.7,false,true,false); // middle goal
//       setGrab(true);
//       ddBackwards(30,false);
// //-32.2 67, turn 180, ,, -32 15
//   tiltDown();      
// // openClamp();

//       // maxSpeed = 6000;
//       // route4();
//       // while (1)
//       //   if (timeSec > 14800)
//       //   {
//       //     clamp.set_value(1);
//       //     stopIntake();
//       //     stop();
//       //     break;
//       //   }
//     }
//     else if (autonChoice == 5) // Goal Rush RIGHT
//     {
//       grabber.set_value(0);
//       clamp.set_value(1);
//       while (true)
//       { // grab first goal
      
//         left_move(-12000);
//         right_move(12000);

//         if (grabberDistance.get() < grabberDistanceValue)
//         { // if sees goal. clamp and leave
//           grabber.set_value(1);
//           left_move(12000);
//           right_move(-12000);
//           break;
//         }
//         if (pos_y > 65)
//         { // if we are beyond the zone, go back so we don't run into the opposing team
//           grabber.set_value(1);
//           left_move(12000);
//           right_move(-12000);
//           break;
//         }

//         pros::delay(10);
//       }
//       liftStop();
//       setLiftHold();
//       while (true)
//       { // REVERSE
//         left_move(12000);
//         right_move(-12000);
//         if (pos_y < 5)
//         { // if we are beyond the zone, go back so we don't run into the opposing team
//           stop();
//           break;
//         }
//         pros::delay(10);
//       }
//       stop();
//     }
//     else if (autonChoice == 6) //  Goal Rush (LEFT)
//     {
//       setGrab(false);
//       goalCover.set_value(1); GOALCOVERDOWN = true;
//       while (true)
//       { // grab first goal
//         liftNoLoop(390);
//         left_move(-12000);
//         right_move(12000);

//         if (grabberDistance.get() < grabberDistanceValue)
//         { // if sees goal. clamp and leave
//           setGrab(true);
//           left_move(12000);
//           right_move(-12000);
//           break;
//         }
//         if (pos_y > 50)
//         { // if we are beyond the zone, go back so we don't run into the opposing team
//           setGrab(true);
//           break;
//         }

//         pros::delay(10);
//       }
//          while (true)
//       { // grab first goal
//         liftNoLoop(390);
//         left_move(12000);
//         right_move(-12000);

//         if (pos_y <  24)
//         { // if we are beyond the zone, go back so we don't run into the opposing team
//           stop();
//           break;
//         }

//         pros::delay(10);
//       }

//       while(true){
//         if(pos_y > 28){
//           liftNoLoop(390);
//           left_move(12000);
//           right_move(-12000);
//         } else{
//           stop(); 
//         }
//       }      
//     }
//     else if (autonChoice == 7) // tests
//     {
//       // park();
//       // setGrab(false);
//       // goalCover.set_value(1); GOALCOVERDOWN = true;
//       // while (true)
//       // { // grab first goal
//       //   liftNoLoop(390);
//       //   left_move(-12000);
//       //   right_move(12000);

//       //   if (grabberDistance.get() < grabberDistanceValue)
//       //   { // if sees goal. clamp and leave
//       //     setGrab(true);
//       //     left_move(12000);
//       //     right_move(-12000);
//       //     break;
//       //   }
//       //   if (pos_y > 50)
//       //   { // if we are beyond the zone, go back so we don't run into the opposing team
//       //     setGrab(true);
//       //     break;
//       //   }

//       //   pros::delay(10);
//       // }
//       //    while (true)
//       // { // grab first goal
//       //   liftNoLoop(390);
//       //   left_move(12000);
//       //   right_move(-12000);

//       //   if (pos_y <  21)
//       //   { // if we are beyond the zone, go back so we don't run into the opposing team
//       //     stop();
//       //     break;
//       //   }

//       //   pros::delay(10);
//       // }
      
//       // setGrab(1);
//       // liftTo(660);
//       // setLiftHold(); liftStop();
//       // closeClamp();
//       // pros::delay(200);
//       // tiltUp();
//       // pros::delay(200);

//       // ddStraight(24,false);
//       // turnPID(90,1);
//       // ddBackwards(24,false);
//       // turnPID(0,1);
//       // stop();
//       // turnToPoint(-20,-20);
//       // // turnToPoint(0,1000);
//       // // turnToPoint(-20,20);
//       // setGrab(false); GRABNGO = true;
//       // continuousArcPID(0,30,0,50,false,true,false); 
//       // GRABNGO = false;
//       // liftUp(6000);
//       // arcMovementPID(20,80,false,false,false);

//       // // arcMovementPID(40, 20, false, false, false);
//       // // // turnPID(0,1);
//       // arcMovementPID(25, 40, false, false, false);
//       // arcMovementPID(-10, -10, true,false, false);


//       // GREATEST_RADIUS = 0;
//       // arcMovementPID(20, 40, false, false, false);
//       // GREATEST_RADIUS = 0;
//       // arcMovementPID(10, 50, false, false, false);
//       // GREATEST_RADIUS = 0;
//       // arcMovementPID(0, 60, false, false, false);
//       // GREATEST_RADIUS = 0;

//       // stop();
//       // std::cout << "Greatest Radius: " << GREATEST_RADIUS;
//     }
//     else if (autonChoice == 8)
//     {
//       // arcMovementPID(0,-80,true,false,true);
//       // turnPID(90,1);
    


//       while(true){
//           char buffer[100];
//           sprintf(buffer, "AutonDisplaY\nX: %f\nY: %f\nT: %f", pos_x, pos_y, theta * 180 / M_PI);
//           lv_label_set_text(AutonDisplay, buffer);
//       pros::delay(10);
//       }
//     }
//     else if (autonChoice == 9)
//     {
//               SPEEDRATIOPID = 0.45;
//               openClamp();
//               setGrab(0);// opens 
//               tiltDown();
//               pros::delay(500); // open up both pistons and wait for a bit.
//               arcMovementPID(0, -6, true, false, true);
//               closeClamp();
//               pros::delay(200);
//               tiltUp();

//               SPEEDRATIO = 0.35;
//               SR1();
//               grabber.set_value(1);
//               liftTo(1348);
//               SPEEDRATIO = 0.40;

//               setLiftHold(); // grab yellow 1
//               SPEEDRATIO = 0.25;

//               arcMovementPID(82, -26.4, false, true, false);
//               stop();
//               openClamp();
//               tiltDown();
//               SPEEDRATIO = 0.35;

//               // pros::delay(200);/
//               maxSpeedPID = 6000;
//               SPEEDRATIO = 0.28;
//               arcMovementPID(100, -37, false, false, false);
//               stop(); // liftup and platform y1
//               maxSpeedPID = 9000;

//               maxSpeedLift = 3000;
//               liftTo(880);
//               maxSpeedLift = 10000;
//               pros::delay(500);
//               grabber.set_value(0); // wait for goal to balance and backup
//               liftUp(4000);
//               pros::delay(250);
//               for (int i = 0; i < 30; i++)
//               {
//                 left_move(6000);
//                 right_move(-6000);
//                 pros::delay(10);
//               }
//               stop();
//               liftTo(390); // go back down

//                         turnPID(303, 1); // grab b1
//                         arcMovementPID(80, -25.4, false, true, false);
//                         stop();
//                         grabber.set_value(1);
//                         liftTo(600);
//                         setLiftHold();
//                         liftStop();
//                         // turnPID(0, 2);
//                         turnPID(120, 1);
//                         liftTo(1100);
//                         maxSpeedPID = 8500; // platorm b1
//                         arcMovementPID(100, -37, false, false, false);
//                         stop();
//                         grabber.set_value(0);
//                         maxSpeedPID = 9000;
//                         // liftUp(-3000);
//                         SPEEDRATIOPID = 0.42;
//                         turnPID(180,1);
//                         arcMovementPID(95.8, 13.5, true, false, true);
//                         pros::delay(200);
//                         clamp.set_value(0); // grab r1
//                         pros::delay(200);
//                         tiltUp();
//                         // for(int i = 0; i < 20; i++){
//                         //   liftNoLoop(390);
//                         //   pros::delay(10);
//                         // }
//                         // SPEEDRATIOPID = 0.46;
//                         liftTo(390);
//                         arcMovementPID(67, -36, false, true, false);
//                         stop();
//                         grabber.set_value(1); // mid
//                         LIFTMIDDLE = true;
//                         liftTo(550); setLiftHold(); liftStop(); // set goal above
//                         turnPID(270, 4);
                        
//                         SPEEDRATIOPID = 0.30;
//                         arcMovementPID(41, -45, false, false, false);
//                         stop();
//                         clamp.set_value(1); // drop r1
//                         tiltDown();

//                         liftTo(1150);
//                         LIFTMIDDLE = false;
//                         arcMovementPID(21.8, -45, false, false, false);     
//                         turnPID(270,1);    
//                         // ddStraight(19.2,false);
//                         SPEEDRATIOPID = 0.42;

//                         stop(); // platform middle goal
//                         maxSpeedLift = 3000;
//                         liftTo(870); // lowest ou can go on the platform
//                         maxSpeedLift = 10000;
//                         pros::delay(500);
//                         grabber.set_value(0);
//                         left_move(3000);
//                         right_move(-3000);
//                         pros::delay(300);
//                         liftUp(5000);
//                         pros::delay(100); // backup while lifting up
//                         for (int i = 0; i <30; i++)
//                         {
//                           left_move(6000);
//                           right_move(-6000);
//                           pros::delay(10);
//                         }
//                         stop();
//                         liftTo(390);
//                         turnPID(110, 1.4);
//                               SPEEDRATIOPID = 0.3;
//                           GRABNGO = true;
//                         arcMovementPID(50, -44, false, true, false);
//                       SPEEDRATIOPID = 0.46;

//                       GRABNGO = false;

//                         stop();
//                         grabber.set_value(1); // grab r1
//                         liftTo(550);
//                         turnPID(265, 1);
//                         liftTo(1100);
//                         arcMovementPID(21, -45, false, false, false);
//                         pros::delay(250);
//                         grabber.set_value(0); // platform r1
//                         liftUp(1100);
//                         for (int i = 0; i < 40; i++)
//                         {
//                           left_move(6000);
//                           right_move(-6000);
//                           pros::delay(10);
//                         }
//                         stop();
//                         liftTo(390); // go back down

//                         turnPID(0, 1);

//                         setGrab(false);

//                 // turnPID(90,1);
//                 // ddStraight(24,false);
//                 // turnPID(0,1);
//                 SPEEDRATIOPID = 0.43;
//                 arcMovementPID(24.8, -101, true, false, true);
//                 //  for (int i = 0; i < 200; i++)
//                 //   {
//                 //     left_move(3000);
//                 //     right_move(-3000);
//                 //     pros::delay(10);
//                 //   }
//                 clamp.set_value(0); // get b2
//                 pros::delay(200);
//                 tiltUp();
//                 pros::delay(200);

//                 SPEEDRATIOPID = 0.28;
//                 arcMovementPID(55, -79, false, true, false);
//                 SPEEDRATIOPID = 0.43;
//                 grabber.set_value(1);
//                 liftTo(660);setLiftHold(); liftStop(); // get y3
//                 turnPID(66, 1);
//                  liftTo(1140);

//                 arcMovementPID(84, -65, false, false, false);
//                 stop();
//                 clamp.set_value(1); // dropb2
//                 tiltDown();
//                 liftTo(1140);
//                 arcMovementPID(97, -58, false, false, false);
//                 ddStraight(2,false);
//                 maxSpeedLift = 3000;
//                 liftTo(1050);
//                 maxSpeedLift = 9000;
//                 grabber.set_value(0); // platform y3
//                 for (int i = 0; i < 25; i++)
//                 {
//                   left_move(6000);
//                   right_move(-6000);
//                   pros::delay(10);
//                 }
//                 stop();
//                 liftTo(390);
//                 turnPID(238.5, 1);
//                 arcMovementPID(84.87, -64, false, true, false);
//                 grabber.set_value(1);
//                 liftTo(600);  setLiftHold(); liftStop();// get b2
//                 turnPID(50, 1);
//                 liftTo(1100);
//                 arcMovementPID(99, -48.7, false, false, false);
//                 turnPID(70,1);
//                 grabber.set_value(0);
//                 pros::delay(300);
//                 // maxSpeedLift = 3000; liftTo(1867); maxSpeedLift = 9000;
//                 for (int i = 0; i < 40; i++)
//                 {
//                   left_move(6000);
//                   right_move(-6000);
//                   pros::delay(10);
//                 }
//                 stop();
//                 liftTo(390);
//     }
    stop();
    std::cout << "AUTON END" << std::endl;
    updateOdom.remove(); // shutsdown the odom thread.
    updateTime.remove();
  }
