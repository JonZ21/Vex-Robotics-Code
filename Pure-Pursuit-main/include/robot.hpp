#ifndef ROBOT_HPP
#define ROBOT_HPP


extern int RESTING_LIFT_HEIGHT;
extern bool GOALCOVERDOWN;
extern bool TILTUP;
void drive();
void left_move(double speed);
void right_move(double speed);
void setBreakTypeBreak();
void setBreakTypeHold();
void setBreakTypeCoast();

extern void tiltUp();
extern void tiltDown();
extern void slowTiltDown();
void stop();
extern void liftOP();
extern void toggleLift();
extern void transmissionOP();
extern void pistons();
extern void intakeOP();
extern int intakeSpeed;
extern int frontGrabberValue;
extern bool CLAMPUP;
extern void climbToggle();
extern void set6MDrive();
extern void set4MDrive();

#endif
