#ifndef AUTON_HPP
#define AUTON_HPP


extern void setClamp(bool value);
extern void setGrab(bool value);
extern bool GRABBED;
extern void setTilter(bool value);
extern void softTiltDrop();
extern void openClamp();
extern void closeClamp();
extern int AUTO_ID;

extern void extraComponents();
extern int timeSec;
extern int grabberDistanceValue;
extern void liftTo(int target);
extern void liftNoLoop(int target);
extern void liftUp(int voltage);
extern void liftDown();
extern void liftStop();
extern void goToLowStack();
extern bool atLowStack();
extern void goToHighStack();
extern bool atHighStack();
extern void frontGrabberAuton();
extern void clampAuton();
extern bool clamped();
extern bool grabbed();
extern void park();
extern void setLiftBrake();
extern void setLiftHold();
extern bool sixMotorDrive;
extern void runIntake();
extern void stopIntake();



#endif
