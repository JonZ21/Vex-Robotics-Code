#ifndef MOVEMENT_HPP
#define MOVEMENT_HPP
#include "vector"

extern bool STRAIGHTARCMOVEMENT;
extern bool FAILSAFEEXCEPTION;
extern bool ARCTURNEXECPTION;
extern int maxSpeedPID;
extern int maxSpeed;
extern double SPEEDRATIOPID;
extern double SPEEDRATIO;
extern double GREATEST_RADIUS;
extern bool PPGRABBERTOGGLE;
extern bool PPCLAMPTOGGLE;
extern double TURNPIDKP;
extern bool GRABNGO;
extern void turnToPoint(double x, double y,bool backwards);

extern void turnToPoint(double x, double y,bool backwards, int liftHeight);

extern void turnPID(double targetTheta, double goals);

class MOVEMENT{
  double m_straightkP;
  double m_straightkI;
  double m_straightkD;
  double m_turnkP;
  double m_turnkI;
  double m_turnkD;
  double m_maxSpeed;
  int numberOfMogos;
public:
  MOVEMENT();
  MOVEMENT(double skp, double ski, double skd, double tkp, double tki, double tkd, double maxSpeed);
  void goToPoint(double targetX, double targetY);
};
class Point{
double x_coord;
double y_coord;
public:
  void setX(double x){ x_coord = x;}
  void setY(double y){ y_coord = y;}
  double getX(){
    return x_coord;
  }
  double getY(){
    return y_coord;
  }
};

class CurvePoint{
  double x;
  double y;
  double moveSpeed;
  double turnSpeed;
  double followDistance;
  double pointLength;
  double slowDownTurnRadians;
  double slowDownTurnAmount;
public:
  CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double slowDownTurnRadians,
                      double slowDownTurnAmount);
  CurvePoint(const CurvePoint &thisPoint);
  Point toPoint();
  void setPoint(Point point);
  double getFollowDistance();
  double getX();
  double getY();
};

void route();
void circleTurnToPoint(double xPoint, double yPoint);
void followCurve(std::vector<CurvePoint> allPoints, double followAngle,bool backwards);
void arcMovement(double xPoint, double yPoint);
void arcMovementPID(double xPoint, double yPoint,bool backwards, bool grab, bool clamp);
void arcMovementPID(double xPoint, double yPoint, bool backwards, bool grab, bool clamp, int liftHeight);
extern int maxSpeedTurn;
#endif
