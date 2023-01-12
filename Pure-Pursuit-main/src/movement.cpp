#include "movement.hpp"
#include "config.hpp"
#include "robot.hpp"
#include "odom.hpp"
#include "main.h"
#include "api.h"
#include "math.h"
#include "PID.hpp"
#include "thread"
#include "display/lvgl.h"
#include "initialize.hpp"
#include "auton.hpp"

bool FAILSAFEEXCEPTION = false;
bool ARCTURNEXECPTION = false;

double angleWrap(double angle)
{ // makes sure an angle is within plusminus pi
  while (angle < -M_PI)
  {
    angle += M_2_PI;
  }
  while (angle > M_PI)
  {
    angle -= M_2_PI;
  }
  return angle;
}

double TURNPIDKP = 380;
int maxSpeedTurn = 11000;


void turnToPoint(double x, double y, bool backwards, int liftHeight){

  toggle = true;
  turnIntegral = 0;
  double failsafe = 0;
  int failsafeCounter = 0;
  int counter = 0;
  double goals = 1;

  double targetTheta = 0;

  double itargetTheta = atan2f(x - pos_x,y - pos_y)*180/M_PI;
  if(backwards)
    itargetTheta += 180;
  std::cout<<targetTheta;
  
  while (1)
  {
    double headingStart;
    liftNoLoop(liftHeight);
    if (toggle == true)
    {
      headingStart = innerAVG();
      std::cout << "turnStart " << headingStart << std::endl;
      std::cout << "turnTarget " << itargetTheta << std::endl;
      toggle = false;
    }

    turnError = (itargetTheta*M_PI/180 - theta);
    turnError = atan2f(sinf(turnError), cosf(turnError)) * 180 / M_PI; // -5.02
  
    turnIntegral = turnIntegral + turnError;
    if (turnError <= 0)
    {
      turnIntegral = 0;
    }
    if (turnIntegral > 2000)
    {
      turnIntegral = 0;
    }
    double turnDerivative = turnError - turnPreviousError;
    if(ARCTURNEXECPTION && fabs(turnError) < 5){
      std::cout<<"Exception turn Break" << std::endl;
      break;
    }
    if ((turnError < 3) && (turnError > -3))
    {
      turnCount += 1;
    }
    else
    { // If It isn't within +/- 0.5
      turnCount = 0;
    }
    if(turnCount >= 1 && x == 23.01 ){
      toggle = true;
      turnCount = 0;
      stop();
      break;
    }
    if (turnCount >= 8)
    {
      toggle = true;
      turnCount = 0;
      stop();
      break;
    }

    double mVrequest = turnError * (TURNPIDKP) * goals + turnDerivative * 2000; // + turnIntegral * p_kI;

    if (mVrequest > maxSpeedTurn)
    {
      left_move(-maxSpeedTurn);
      right_move(-maxSpeedTurn);
    }
    else if (mVrequest < -maxSpeedTurn)
    {
      left_move(maxSpeedTurn);
      right_move(maxSpeedTurn);
    }
    else
    {
      left_move(-mVrequest);
      right_move(-mVrequest);
    }
        std::cout << "turnError: " << turnError << " theta: " << theta << " MV: " << mVrequest << std::endl;

    turnPreviousError = turnError;
    if (failsafeCounter % 50 == 0)
    {
      if (fabs(failsafe - turnError) < 5)
      {
        std::cout << "turn broke" << std::endl;
        break;
      }

      failsafe = turnError;
    }
    failsafeCounter++;
    // odom(); counter++;
    pros::delay(10);
  }
}

void turnToPoint(double x, double y, bool backwards){
  
  turnToPoint(x,y,backwards,lift.get_position());
}

void turnPID(double targetTheta, double goals)
{

  toggle = true;
  turnIntegral = 0;
  double failsafe = 0;
  int failsafeCounter = 0;
  int counter = 0;
  if(clamped() && !grabbed())
  {
    goals+= 0.01;
  }
  if(grabbed() && !grabbed()){
    goals+= 0.01;
  }
  if(grabbed() && clamped()){
    goals+= 0.015;
  }

  while (1)
  {
    double headingStart;
    if (toggle == true)
    {
      headingStart = innerAVG();
      std::cout << "turnStart " << headingStart << std::endl;
      std::cout << "turnTarget " << targetTheta << std::endl;
      toggle = false;
    }

    turnError = (targetTheta*M_PI/180 - theta);
    turnError = atan2f(sinf(turnError), cosf(turnError)) * 180 / M_PI; // -5.02
  
    turnIntegral = turnIntegral + turnError;
    if (turnError <= 0)
    {
      turnIntegral = 0;
    }
    if (turnIntegral > 2000)
    {
      turnIntegral = 0;
    }
    double turnDerivative = turnError - turnPreviousError;

    if(ARCTURNEXECPTION && fabs(turnError) < 5){
      std::cout<<"Exception turn Break" << std::endl;
      break;
    }
    if ((turnError < 3) && (turnError > -3))
    {
     
      turnCount += 1;
    }
    else
    { // If It isn't within +/- 0.5
      turnCount = 0;
    }
    if (turnCount >= 8)
    {
      toggle = true;
      turnCount = 0;
      stop();
      break;
    }

    double mVrequest = turnError * (TURNPIDKP) * goals + turnDerivative * 2000; // + turnIntegral * p_kI;

    if (mVrequest > maxSpeedTurn)
    {
      left_move(-maxSpeedTurn);
      right_move(-maxSpeedTurn);
    }
    else if (mVrequest < -maxSpeedTurn)
    {
      left_move(maxSpeedTurn);
      right_move(maxSpeedTurn);
    }
    else
    {
      left_move(-mVrequest);
      right_move(-mVrequest);
    }
        std::cout << "turnError: " << turnError << " theta: " << theta << " MV: " << mVrequest << std::endl;

    turnPreviousError = turnError;
    if (failsafeCounter % 50 == 0)
    {
      if (fabs(failsafe - turnError) < 4)
      {
        std::cout << "turn broke" << std::endl;
        break;
      }

      failsafe = turnError;
    }
    failsafeCounter++;
    // odom(); counter++;
    pros::delay(10);
  }
}

CurvePoint::CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double slowDownTurnRadians,
                       double slowDownTurnAmount)
{
  this->x = x;
  this->y = y;
  this->moveSpeed = moveSpeed;
  this->turnSpeed = turnSpeed;
  this->followDistance = followDistance;
  this->slowDownTurnRadians = slowDownTurnRadians;
  this->slowDownTurnAmount = slowDownTurnAmount;
  // POintLneght??
}
CurvePoint::CurvePoint(const CurvePoint &thisPoint)
{ // copy constructor... but it takes the address of the obect?
  x = thisPoint.x;
  y = thisPoint.y;
  moveSpeed = thisPoint.moveSpeed;
  turnSpeed = thisPoint.turnSpeed;
  followDistance = thisPoint.followDistance;
  slowDownTurnRadians = thisPoint.slowDownTurnRadians;
  slowDownTurnAmount = thisPoint.slowDownTurnAmount;
  pointLength = thisPoint.pointLength;
};

Point CurvePoint::toPoint()
{
  Point newPoint;
  newPoint.setX(x);
  newPoint.setY(y);
  return newPoint;
}

void CurvePoint::setPoint(Point point)
{
  x = point.getX();
  y = point.getY();
}

double CurvePoint::getFollowDistance()
{
  return followDistance;
}
double CurvePoint::getX()
{
  return x;
}
double CurvePoint::getY()
{
  return y;
}

int loops = 0;
std::vector<Point> lineCricleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2)
{
  // This finds if there are two points that are super close together, and just makes them one?
  // std::cout<<"line: " <<loops << std::endl;

  if (fabs(linePoint1.getY() - linePoint2.getY()) < 0.003)
  { // he has it as cm, we're usin ginches for now/
    linePoint1.setY(linePoint2.getY() + 0.003);
  }

  if (fabs(linePoint1.getX() - linePoint2.getX()) < 0.003)
  { // he has it as cm, we're usin ginches for now/
    linePoint1.setX(linePoint2.getX() + 0.003);
  }

  double m1 = (linePoint2.getY() - linePoint1.getY()) / (linePoint2.getX() - linePoint1.getX());

  double quadraticA = 1.0 + pow(m1, 2);

  double b = (linePoint1.getY()) - m1 * (linePoint1.getX());

  double quadraticB = (-2 * pos_x) + (2.0 * m1 * b) - (2 * pos_y * m1);

  double quadraticC = pow(pos_x, 2) + pow(b, 2) - (2 * b * pos_y) + pow(pos_y, 2) - pow(radius, 2);

  std::vector<Point> allPoints;

  // std::cout<<"SLOPE: " << m1 << " QUADRATICS: " << quadraticA << ", " << quadraticB << ", " << quadraticC<<std::endl;

  double minX;
  double maxX;
  if (linePoint1.getX() < linePoint2.getX())
  {
    minX = linePoint1.getX();
    maxX = linePoint2.getX();
  }
  else
  {
    maxX = linePoint1.getX();
    minX = linePoint2.getX();
  }

  //  std::cout<<"MIN: " << minX << " MAX: " << maxX << std::endl;

  try
  {
    double xRoot1 = (-quadraticB + sqrtf(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
    double yRoot1 = m1 * (xRoot1) + b;

    // bounding box equation. We wont want points outside of the bounding box. We don't know which order of the points are on.
    if (xRoot1 > minX && xRoot1 < maxX)
    { // found a point on the line
      Point newPoint;
      newPoint.setX(xRoot1);
      newPoint.setY(yRoot1);
      allPoints.push_back(newPoint);
      // std::cout<<"xRoot1 found point: (" << newPoint.getX()  << ", " << newPoint.getY() << ") " << std::endl;
    }
    else
    {
    } // std::cout<<"xRoot1 no points" <<std::endl;

    double xRoot2 = (-quadraticB - sqrtf(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
    double yRoot2 = m1 * (xRoot2) + b;

    if (xRoot2 > minX && xRoot2 < maxX)
    { // found a point on the line
      Point newPoint;
      newPoint.setX(xRoot2);
      newPoint.setY(yRoot2);
      allPoints.push_back(newPoint);
      // std::cout<<"xRoot2 found point: (" << newPoint.getX()  << ", " << newPoint.getY() << ") " << std::endl;
    }
    else
    {
    } // std::cout<<"xRoot2 no points" <<std::endl;
  }
  catch (std::exception e)
  {

    // there is no intersection found. (Maybe we can make the radius HUGE? )
  }
  return allPoints;
}

// gets the instantious point for the robot to follow.
// this is default going to the first point. (Edge cases are not included.)
CurvePoint getFollowPointPath(std::vector<CurvePoint> pathPoints, Point robotLocation, double followRadius)
{
  CurvePoint followMe(pathPoints.at(0));
  std::vector<Point> intersections;
  std::vector<Point> intersections2;

  for (int i = 0; i < pathPoints.size() - 1; i++)
  { // looking through the points. Prefers the points that are later in the list
    CurvePoint startLine = pathPoints.at(i);
    CurvePoint endLine = pathPoints.at(i + 1);

    // end needs to be fix ed

    intersections = lineCricleIntersection(robotLocation, pathPoints.at(i).getFollowDistance(), startLine.toPoint(), endLine.toPoint());

    if (intersections.size() == 1)
    {
      followMe.setPoint(intersections.at(0));
    }
    else if (intersections.size() == 2)
    {
      Point one = intersections.at(0);
      Point two = intersections.at(1);

      double distanceOne = sqrtf(pow((endLine.getX() - one.getX()), 2) + pow((endLine.getY() - one.getY()), 2));
      double distanceTwo = sqrtf(pow((endLine.getX() - two.getX()), 2) + pow((endLine.getY() - two.getY()), 2));

      if (distanceOne < distanceTwo)
      {
        followMe.setPoint(one);
      }
      else
      {
        followMe.setPoint(two);
      }
    }
  }

  return followMe;
}
double gayasscirclearcthing(double xPoint, double yPoint)
{
  // This function recieves a point and calculates the radius beteen the point
  // current position such that both the point and robot are points on the circle
  // Normalize the Point such that Robot Position is at 0,0
  double xPos = xPoint - pos_x;
  double yPos = yPoint - pos_y;

  // Calculate halfway poing between two points
  double halfwayX = xPos / 2;
  double halfwayY = yPos / 2;

  // Calculate intersection line slope and b
  double intersectionLineSlope = (yPoint - pos_y) / (xPoint - pos_x);
  double intersectionLineb = 0;

  // Calculate slope and b of perpendicular line at halfway point
  double halfwaySlope = 1 / (intersectionLineSlope * -1);
  double halfwayb = halfwayY - halfwaySlope * halfwayX;

  // Calculate the intersection between the perpendicular line and the x axis
  double radius = -1 * halfwayb / halfwaySlope;

  return fabs(radius);
}

int maxSpeed = 9000;   double SPEEDRATIO = 0.45;

bool PPGRABBERTOGGLE = false;
bool PPCLAMPTOGGLE = false;
bool STRAIGHTARCMOVEMENT = false;

void arcMovement(double xPoint, double yPoint, bool backwards)
{ // no sigmoid function, but realistically it should be fine??
  // This function recieves an x,y point and will curve turn to that point
  // Goalpoint coordinate lets us know what the speed should be based on the distance from the robot to the actual goal point.
  int goalsCarried = 1;
  if(clamped())
    goalsCarried++;
  if(grabbed())
    goalsCarried++;

  if(PPGRABBERTOGGLE)
    frontGrabberAuton();
  if(PPCLAMPTOGGLE)
    clampAuton();  

  if (xPoint == 0 && yPoint == 0)
  {
    stop();
  }
  else
  {
    double targetTheta = atan2f(xPoint - pos_x, yPoint - pos_y);
    // std::cout<<"targetTheta:" << targetTheta<< std::endl;
    if (backwards)
    {
      targetTheta = angleWrap(targetTheta + M_PI);
    }

    bool rightTurn = false;
    if (backwards)
    {
      targetTheta = ((targetTheta + M_PI) - theta);
      targetTheta = atan2f(sinf(targetTheta), cosf(targetTheta)) * 180 / M_PI;
    }
    else
    {
      targetTheta = (targetTheta - theta);
      targetTheta = atan2f(sinf(targetTheta), cosf(targetTheta)) * 180 / M_PI;
      //
    }
    // if(backwards)
    //   targetTheta = angleWrap(targetTheta + 180);


    // std::cout << "targetT: " << targetTheta * 180 / M_PI << std::endl;

    if (targetTheta >= 0)
    { // Right Turn
      // std::cout << "rightTurn" << std::endl;
      if (backwards)
        rightTurn = false;
      else
        rightTurn = true;
    }
    else
    { // left turn
      // std::cout << "leftTurn" << std::endl;
      if (backwards)
        rightTurn = true;
      else
        rightTurn = false;

     
    }
    double sRatio = SPEEDRATIO;
 
    bool forward = false;
    for(int i = 0; i < goalsCarried; i++){
      sRatio -= .09;
      // maxSpeed += 1000;
    }
    if(STRAIGHTARCMOVEMENT){
      sRatio = 0.7;
    } 
    if (fabs(targetTheta) < 1.5)
    { // test
      forward = true;
      if (backwards)
      {
        left_move(maxSpeed);
        right_move(-maxSpeed);
      }
      else
      {
        left_move(-maxSpeed);
        right_move(maxSpeed);
      }
    }
    else if (rightTurn)
    {
      forward = false;

      if (backwards)
      {
        left_move(maxSpeed*sRatio);
        right_move(-maxSpeed);
      }
      else
      {
        left_move(-maxSpeed);
        right_move(maxSpeed *sRatio);
      }
    }
    else
    {
      forward = false;
      if (backwards)
      {
        left_move(maxSpeed);
        right_move(-maxSpeed*sRatio);
      }
      else
      {
        left_move(-maxSpeed *sRatio);
        right_move(maxSpeed);
      }
    }
      
      std::cout<<"turnAngle: " << targetTheta << " theta: " << theta*180/M_PI << "straight: " <<  std::endl;

  }
  pros::delay(10);

}

int maxSpeedPID = 9000; 
double SPEEDRATIOPID = 0.48;
double GREATEST_RADIUS = 0;



bool GRABNGO = false;
   // set a ratio between the radius to determine the max speed.
// bool CONTINUOUSEXCEPTION = false;
void arcMovementPID(double xPoint, double yPoint, bool backwards, bool grab, bool clamp){
  arcMovementPID(xPoint,yPoint,backwards,grab,clamp,lift.get_position());
}

void arcMovementPID(double xPoint, double yPoint, bool backwards, bool grab, bool clamp, int liftHeight)
{ // no sigmoid function, but realistically it should be fine??
  // This function recieves an x,y point and will curve turn to that point
  // Goalpoint coordinate lets us know what the speed should be based on the distance from the robot to the actual goal point.

  // Get the center radius of the  to the point

  // error based on distance from point to posx only to determine max speed not for pid anything else okay thanks.
  double maxPID;
  int breakCounter = 0;
  bool rightTurn = false;
  double startError = sqrt(pow(xPoint - pos_x, 2) + pow(yPoint - pos_y, 2));
  double kP = 2200;

  double previousTurnAngle(0);
  bool switched = false;
  if (backwards)
    switched = true;
  double distanceError = startError;
  double previousError = startError;
  int failsafeCounter = 0;
  double failsafeCheck = 0;
  

  bool turnFixToggle = false;
  // Determine if the robot is turning left or rightLift
  while (true)
  {

    if(yPoint == -60.01 && pos_y < -60){
      break;
    }
    if(xPoint  == 55.01 && pos_x > 55){
      break;
    }
    if(AUTO_ID == 1){
      if(pos_x > 60){
        maxSpeedPID = 4000;
      }
    }
    if (AUTO_ID == 2){
      if(pos_y < -40)
        maxSpeedPID = 7000;
    }

    if( AUTO_ID == 3){
       if(pos_x < 60 && pos_x > 24)
        maxSpeedPID = 7000;
      else 
        maxSpeedPID = 11000;
    }
    
    liftNoLoop(liftHeight);
    int goalsCarried = 1;
    if(clamped())
      goalsCarried++;
    if(grabbed())
      goalsCarried++;
    
    if(GRABNGO && grabbed()){
      stop();
      break;
    }

    //  route1extras(xPoint, yPoint);
    if(grab){
      frontGrabberAuton();
    }
    // if(grab && grabbed())
    //   break;

    // if(clamp && clamped()) clamp is a funky business
    
    // if(clamp){
    //   clampAuton();

    // }

    distanceError = sqrt(pow(xPoint - pos_x, 2) + pow(yPoint - pos_y, 2));
    double speed = kP * distanceError;
    if (fabs(speed) > fabs(maxSpeedPID))
    {
      speed = maxSpeedPID;
    }
    if(!FAILSAFEEXCEPTION){
      if(failsafeCounter %20 == 0  && yPoint == -98.001){
        if (fabs(fabs(failsafeCheck) - fabs(distanceError)) < 0.5)
        {
          break;
        }
      }
      if (failsafeCounter % 50 == 0)
      {
        if (fabs(fabs(failsafeCheck) - fabs(distanceError)) < 0.5)
        {
          break;
        }
        failsafeCheck = distanceError;
      }
      
    }


    if (previousError < distanceError)
    {
      switched = !switched;
    }
    // else if (previousError >= distanceError){
    //   switched = false;
    // }

    double radius = gayasscirclearcthing(xPoint, yPoint);      // WHat value is this?
    double turnAngle = atan2f(xPoint - pos_x, yPoint - pos_y); // radians
    double targetTheta = atan2f(xPoint - pos_x, yPoint - pos_y);

    if(radius > GREATEST_RADIUS){
      GREATEST_RADIUS = radius;
    }

    if (backwards)
    {
      targetTheta = ((targetTheta + M_PI) - theta);
      targetTheta = atan2f(sinf(targetTheta), cosf(targetTheta)) * 180 / M_PI;
    }
    else
    {
      targetTheta = (targetTheta - theta);
      targetTheta = atan2f(sinf(targetTheta), cosf(targetTheta)) * 180 / M_PI;
      //
    }


    if(!turnFixToggle){
      if(fabs(targetTheta) > 30){
        ARCTURNEXECPTION = true;
        turnToPoint(xPoint,yPoint,backwards,liftHeight);
        failsafeCheck = 0;
        // turnPID(targetTheta,1); should be turn to point
      
        ARCTURNEXECPTION = false;
       }
       turnFixToggle = true;
    }
    std::cout <<"targetT: "<< targetTheta << std::endl;

    double isoAngle = M_PI / 2 - turnAngle;
    double circleAngle = M_PI - (2 * isoAngle);

    // Calculate the left and right side distance to drive
    double leftRadius = 0;
    double rightRadius = 0;

    if (targetTheta >= 0)
    { // Right Turn
      // left side travels more (faster ) than the right side
      rightTurn = true;
      leftRadius = radius + 5.75; // The number is the distance from the Center to Tracker
      rightRadius = radius - 5.75;
    }
    else
    { // left turn
      leftRadius = radius - 5.75;
      rightRadius = radius + 5.75;
      rightTurn = false;
    }

    double leftDistance = leftRadius * circleAngle;
    double rightDistance = rightRadius * circleAngle;
    //
    // std::cout<<"radius: " << radius << "left: " <<leftDistance << "Right: " << rightDistance << std::endl;
    double leftSpeed;
    double rightSpeed;

    double sRatio = SPEEDRATIOPID;
    // double sRatio = 0;
    // sRatio = 0.24769*log(GREATEST_RADIUS) - 0.271314;

    // if(sRatio > 0.9)
    //   sRatio = 0.9;
    
    // if(goalsCarried == 1){
    //   sRatio -= 0.1;
    // } else if (goalsCarried == 2){
    //   sRatio -= 0.2;
    // }
    if (fabs(targetTheta) < 1.5)
    { // test
      std::cout<<"FORWARDS"<<std::endl;
      if (backwards)
      {
        left_move(speed);
        right_move(-speed);
      }
      else
      {
        left_move(-speed);
        right_move(speed);
      }
    }
    else if (rightTurn)
    {
      // std::cout<<"RIGHT"<<std::endl;
      if (backwards)
      {
        left_move(speed*sRatio);
        right_move(-speed);
      }
      else
      {
        left_move(-speed);
        right_move(speed*sRatio);
        //  printf("leftSpeed : %f rightSpeed: %f Theta : %f\n", maxSpeed, maxSpeed*0.8, targetTheta);
      }
    }
    else
    {
      // std::cout<<"LEFT"<<std::endl;
      if (backwards)
      {
        left_move(speed );
        right_move(-speed*sRatio);
      }
      else
      {
        left_move(-speed*sRatio);
        right_move(speed );
      //  printf("leftSpeed : %f rightSpeed: %f Theta : %f\n",maxSpeed*80, maxSpeed, targetTheta);

      }
    }
        // std::cout<<"THETA: " << theta*180/M_PI << " TARGETTHETA: " << targetTheta << std::endl;
      //  std::cout<<"leftSpeed : "<<leftM.get_voltage() <<" rightSpeed: " << rightM.get_voltage() << " Theta : "<<targetTheta  << std::endl;
        //    std::cout<<"distanceError: "<<distanceError <<"LS: "<<leftSpeed <<" RS: " <<rightSpeed <<" X: " << pos_x<< " Y: " << pos_y<< std::endl;
    if (fabs(distanceError) < 4)
      {
        // std::cout<<"IN THE ZONE" << std::endl;
        if(backwards)
          ddBackwards(fabs(distanceError),clamp);
        else
          if(yPoint == -70.01){
            ddStraightNoFail(distanceError, grab);
          } else{
            ddStraight(distanceError,grab);
          }
        break;
      // breakCounter++;
      
      // std::cout << "BreakCounter: " << breakCounter << " X: " << pos_x << " Y: " << pos_y << std::endl;
      // if (grab && breakCounter > 30)
      // {
      //   grabber.set_value(1);
      //   stop();
      //   break;
      // }
    }
    else
    {
      breakCounter = 0;
    }
    //
    // std::cout<<"Distance: " << grabberDistance.get()<< std::endl;
    autonDisplay();
    // odom(); counter++;
    previousTurnAngle = turnAngle;
    previousError = distanceError;
    failsafeCounter++;
    pros::delay(10);
  }
}

double MINSPEEDPID = 6000;
void continuousArcPID(double x1, double y1, double x2, double y2, bool backwards, bool grab, bool clamp){ // assuming that you want to grab 
 double maxPID;
  int breakCounter = 0;
  bool rightTurn = false;
  double startError = sqrt(pow(x1 - pos_x, 2) + pow(y1 - pos_y, 2));
  double kP = 2200;
  double previousTurnAngle(0);
  bool switched = false;
  if (backwards)
    switched = true;
  double distanceError = startError;
  double previousError = startError;
  int failsafeCounter = 0;
  double failsafeCheck = 0;
  bool turnFixToggle = false;
  
  // Determine if the robot is turning left or rightLift
  while (true)
  {
    int goalsCarried = 1;
    if(clamped())
      goalsCarried++;
    if(grabbed())
      goalsCarried++;
    
    if(GRABNGO && grabbed()){
      stop();
      break;
    }

    if(grab){
      frontGrabberAuton();
    }
    distanceError = sqrt(pow(x1 - pos_x, 2) + pow(y1 - pos_y, 2));
    double speed = kP * distanceError;
    if (fabs(speed) > fabs(maxSpeedPID))
    {
      speed = maxSpeedPID;
    }
    if (fabs(speed) < fabs(MINSPEEDPID))
    {
      speed = MINSPEEDPID;
    }
    if(!FAILSAFEEXCEPTION){
      if (failsafeCounter % 50 == 0)
      {
        if (fabs(fabs(failsafeCheck) - fabs(distanceError)) < 0.5)
        {
          break;
        }
        failsafeCheck = distanceError;
      }
    }


    if (previousError < distanceError)
    {
      switched = !switched;
    }

    double turnAngle = atan2f(x1 - pos_x, y1 - pos_y); // radians
    double targetTheta = atan2f(x1 - pos_x, y1 - pos_y);

    if (backwards)
    {
      targetTheta = ((targetTheta + M_PI) - theta);
      targetTheta = atan2f(sinf(targetTheta), cosf(targetTheta)) * 180 / M_PI;
    }
    else
    {
      targetTheta = (targetTheta - theta);
      targetTheta = atan2f(sinf(targetTheta), cosf(targetTheta)) * 180 / M_PI;
      //
    }


    if(!turnFixToggle){
      if(fabs(targetTheta) > 30){
        ARCTURNEXECPTION = true;
        turnToPoint(x1,y1,backwards);      
        ARCTURNEXECPTION = false;
       }
       turnFixToggle = true;
    }
    std::cout <<"targetT: "<< targetTheta << std::endl;

    double isoAngle = M_PI / 2 - turnAngle;
    double circleAngle = M_PI - (2 * isoAngle);

    // Calculate the left and right side distance to drive
  
    if (targetTheta >= 0)
    { // Right Turn
      // left side travels more (faster ) than the right side
      rightTurn = true;

    }
    else
    { // left turn
      rightTurn = false;
    }

    double leftSpeed;
    double rightSpeed;

    double sRatio = SPEEDRATIOPID;
    if (fabs(targetTheta) < 1.5)
    {
      if (backwards)
      {
        left_move(speed);
        right_move(-speed);
      }
      else
      {
        left_move(-speed);
        right_move(speed);
      }
    }
    else if (rightTurn)
    {
      std::cout<<"RIGHT"<<std::endl;
      if (backwards)
      {
        left_move(speed*sRatio);
        right_move(-speed);
      }
      else
      {
        left_move(-speed);
        right_move(speed*sRatio);
         printf("leftSpeed : %f rightSpeed: %f Theta : %f\n", maxSpeed, maxSpeed*0.8, targetTheta);
      }
    }
    else
    {
      std::cout<<"LEFT"<<std::endl;
      if (backwards)
      {
        left_move(speed );
        right_move(-speed*sRatio);
      }
      else
      {
        left_move(-speed*sRatio);
        right_move(speed );
       printf("leftSpeed : %f rightSpeed: %f Theta : %f\n",maxSpeed*80, maxSpeed, targetTheta);

      }
    }
    if (fabs(distanceError) < 4)
      {
        // std::cout<<"IN THE ZONE" << std::endl;
        double temp = maxSpeedPID;
        maxSpeedPID = MINSPEEDPID;
        arcMovementPID(x2,y2,backwards,grab,clamp);
        maxSpeedPID = temp;
        break;
    }
    else
    {
      breakCounter = 0;
    }
    autonDisplay();
    previousTurnAngle = turnAngle;
    previousError = distanceError;
    failsafeCounter++;
    pros::delay(10);
  }
}
void followCurve(std::vector<CurvePoint> allPoints, double followAngle, bool backwards)
{

  Point robotPosition;
  robotPosition.setX(pos_x);
  robotPosition.setY(pos_y);

  CurvePoint followMe = getFollowPointPath(allPoints, robotPosition, allPoints.at(0).getFollowDistance());
  // Here we are using a fixed follow distance, this can be changed to a dynamic one. (15:50 tutorial 5)

  // std::cout<< "FollowMe: x: " << followMe.getX() << " y: " << followMe.getY() << " Current X: "<< pos_x << "Y: " << pos_y << std::endl;
  //
  // pros::lcd::print(0,"x: %f",pos_x);
  // pros::lcd::print(1,"y: %f",pos_y);
  // pros::lcd::print(2,"target x: %f",followMe.getX());
  // pros::lcd::print(3,"target y: %f",followMe.getY());
  // Debugg
  if (backwards)
  {
    arcMovement(followMe.getX(), followMe.getY(), true); // we need to make sure if its the final point
  }
  else
  {
    arcMovement(followMe.getX(), followMe.getY(), false); // we need to make sure if its the final point
  }
}

int getQuadrant(double angle)
{
  if (angle <= M_PI / 2)
    return 1;
  else if (angle > M_PI / 2 && angle <= M_PI)
    return 4;
  else if (angle > M_PI && angle <= 3 * M_PI / 2)
    return 3;
  else if (angle > 3 * M_PI / 2 && angle <= 2 * M_PI)
    return 2;
  return 0;
  // switch(initialQuadrant){
  //   case 1:
  //     left_Initialx = pos_x - h * cos(theta);
  //     left_Initialy = pos_y + h * sin(theta);
  //     right_Initialx = pos_x + h * cos(theta);
  //     right_Initialy = pos_y - h * sin(theta);
  //     break;
  //   case 2:
  //     left_Initialx = pos_x - h * cos(theta);
  //     left_Initialy = pos_y - h * sin(theta);
  //     right_Initialx = pos_x + h * cos(theta);
  //     right_Initialy = pos_y + h * sin(theta);
  //     break;
  //   case 3:
  //     left_Initialx = pos_x + h * cos(theta);
  //     left_Initialy = pos_y - h * sin(theta);
  //     right_Initialx = pos_x - h * cos(theta);
  //     right_Initialy = pos_y + h * sin(theta);
  //     break;
  //   case 4:
  //     left_Initialx = pos_x + h * cos(theta);
  //     left_Initialy = pos_y + h * sin(theta);
  //     right_Initialx = pos_x - h * cos(theta);
  //     right_Initialy = pos_y - h * sin(theta);
  //     break;
  //}
}

void circleTurnToPoint(double xPoint, double yPoint)
{
  // This function recieves an x,y point and will curve turn to that point

  double kp = 1000;
  double kd = 0;
  // Get the center radius of the robot to the point
  double radius = gayasscirclearcthing(xPoint, yPoint);

  // Determine if the robot is turning left or rightLift
  double turnAngle = atan2f(xPoint - pos_x, yPoint - pos_y); // radians
  double isoAngle = M_PI / 2 - turnAngle;
  double circleAngle = M_PI - (2 * isoAngle);

  // Calculate the left and right side distance to drive
  double leftRadius = 0;
  double rightRadius = 0;

  if (turnAngle >= 0)
  {                          // Right Turn
    leftRadius = radius + 5; // The number is the distance from the Center to Tracker
    rightRadius = radius - 5;
  }
  else
  {
    leftRadius = radius - 5;
    rightRadius = radius + 5;
  }

  double leftDistance = leftRadius * circleAngle;
  double rightDistance = rightRadius * circleAngle;
  double offsetL = leftRotationValue() * 1.375 * 1.004 * M_PI / 180;
  double offsetR = rightRotationValue() * 1.375 * 1.004 * M_PI / 180;
  double previousL = offsetL;
  double previousR = offsetR;
  std::cout << "radius: " << radius << "left: " << leftDistance << "Right: " << rightDistance << std::endl;
  int breakCounter = 0;
  double ratioL = leftDistance / rightDistance;
  // PID Drive for LEFT and RIGHT trackers
  while (true)
  {
    double currentL = (offsetL + leftRotationValue() * (1.375) * 1.004 * M_PI / 180);
    double currentR = (offsetR + rightRotationValue() * (1.375) * 1.004 * M_PI / 180);

    std::cout << "Cleft: " << currentL << "CRight: " << currentR << std::endl;
    double errorR = rightDistance - currentR;
    double errorL = leftDistance - currentL;

    double derivativeL = currentL - previousL;
    double derivativeR = currentR - previousR;

    double speedL = errorL * kp + derivativeL * kd;
    double speedR = errorR * kp + derivativeL * kd;

    speedL = speedL * ratioL;
    speedR = speedR * (1 / ratioL);

    left_move(-speedL / 5);
    right_move(speedR / 5);

    previousL = currentL;
    previousR = currentR;

    if ((fabs(errorL) + fabs(errorR)) / 2 < 0.7)
    {
      breakCounter++;
      if (breakCounter > 50)
        break;
    }
    else
    {
      breakCounter = 0;
    }

    odom();
    counter++;
    pros::delay(10);
  }
}
