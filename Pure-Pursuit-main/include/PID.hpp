#ifndef PID_HPP
#define PID_HPP

class PID{ // does this bitch use the same kP, kd, and KI values for turning AND driving????
  double p_kP;
  double p_kI;
  double p_kD;
  double p_target;
  double p_maxSpeed;
  int numberOfMogos;
  public:
    PID(double target){
      p_kP = 1200;
      p_kI = 0;
      p_kD = 1800;
      p_maxSpeed = 12000;
      p_target = target;
    }
    PID(double kP, double kI, double kD, double target, double maxSpeed); // kP, kI, kD, Target(distance), MaxSpeed
    void straightPID();
};

class TURNPID{ // does this bitch use the same kP, kd, and KI values for turning AND driving????
  double p_kP;
  double p_kI;
  double p_kD;
  double p_maxSpeed;
  double p_targetTheta;
  int numberOfMogos;
  public:
    TURNPID(double kP, double kI, double kD, double x, double y, double maxSpeed); // turning
    TURNPID(double kP, double kI, double kD, double targetTheta, double maxSpeed); // turning

    void turnPID();
};


extern bool toggle;
extern bool toggle2;
extern double displacementX;
extern double displacementY;
extern double headingGoal;
extern double headingStart;
extern double headingCont;
extern double headingCart;
extern double turnPreviousError;
extern double turnCount;
extern double errorCount;
extern double turnError;
extern double forwardPreviousError;
extern double distanceCount;
extern double distanceError;
extern double turnIntegral;
extern double turnKP; //turn correction proportional
extern double turnKI; //turn correction integral
extern double turnKD; //turn correction derivative
extern double distancePreviousError;
extern double xTarget;
extern double yTarget;
extern int DDSTRAIGHTMAX;
extern void straightPID(double p_targetX, double p_targetY, double kP_real);
extern void ddBackwards(double target,bool clamp);
extern void ddStraight(double target, bool grab);
extern void ddStraightNoFail(double target, bool grab);
extern void ddBackwardsNoFail(double target, bool clamp);
extern int STRAIGHTPIDKP;


#endif
