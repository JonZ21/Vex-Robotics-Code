#include "config.hpp"
#include "robot.hpp"
#include "main.h"
#include "api.h"
#include "auton.hpp"
bool grabUp = false;
bool CLAMPUP = false;
bool TILTUP = false;
bool tiltToggle = false;
bool slowTilting = false;
int tiltCounter = 0; 
int counterd = 0;
int tiltDelay = 0;
int RESTING_LIFT_HEIGHT = 2900;

bool sixMotorDrive = true;

void left_move(double speed)
{
  leftF.move_voltage(-speed);
  leftM.move_voltage(-speed);
  leftB.move_voltage(-speed);
}
int intakeSpeed = 12000;
bool autoIntakeStop = false;
int caughtTimerOP = 0;
int outakeTimerOP = 0;
bool OPCAUGHT = false;
   
void intakeOP()
{

  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
  {
    intake.move_voltage(-12000);
  } else if(liftPot.get_position() > 13000 && !master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && TILTUP && !autoIntakeStop)
  {
    intake.move_voltage(intakeSpeed);
  }
  else {
    intake.move_voltage(0);
  }

  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
    autoIntakeStop = !autoIntakeStop;
  }

  //  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
  //    intakeSpeed += 500;
  // }
  //    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
  //    intakeSpeed -= 500;
  // }
 
}

void right_move(double speed)
{
  rightF.move_voltage(-speed);
  rightM.move_voltage(-speed);
  rightB.move_voltage(-speed);
}

void stop()
{
  leftF.move_velocity(0);
  leftM.move_velocity(0);
  leftB.move_velocity(0);
  rightF.move_velocity(0);
  rightM.move_velocity(0);
  rightB.move_velocity(0);
}

void setBreakTypeBreak()
{
  leftF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  rightF.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  leftM.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  rightM.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  leftB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  rightB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void setBreakTypeHold()
{
  leftF.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightF.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  leftM.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightM.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  leftB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void setBreakTypeCoast()
{
  leftF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rightF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  leftM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rightM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  leftB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rightB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void liftOP()
{
  double liftSpeed = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) * 12000 / 127;
  lift.move_voltage(-liftSpeed);
  if(liftSpeed == 0 && liftPot.get_position() < 8000){
    liftNoLoop(2850);
  }
}


void toggleLift()
{
  
  // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
  //   // liftPot.reset();
  //   liftPot.reset_position();
  // }
  // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
  // {
  //   if (lift.get_brake_mode() == pros::E_MOTOR_BRAKE_BRAKE)
  //   {
  //     lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  //   }
  //   else if (lift.get_brake_mode() == pros::E_MOTOR_BRAKE_HOLD)
  //   {
  //     lift.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  //   }
  //   master.rumble(".");
  // }
}


void drive()
{
  double lSpeed = (-master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) - master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) * 12000 / 127;
  double rSpeed = (-master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) + master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) * 12000 / 127;
  // double lSpeed = (-master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) * 12000 / 127;
  // double rSpeed = (-master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) * 12000 / 127;
  right_move(rSpeed);
  left_move(lSpeed);
  if(lSpeed == 0 && rSpeed == 0){
    stop();
  }
}

bool intakeTransValue = false;
bool driveTransValue = false;
bool intakeToggle = false;
bool driveToggle = false;
bool doubleToggle = false;
bool startDelay = false;
bool autoIntake = false;
int transmissionDelay = 0;
void transmissionOP(){


  if(startDelay){
    transmissionDelay++;
    if(transmissionDelay > 30){
      transmissionDelay = 0;
      startDelay = false;
      autoIntake = true;
      
    }
  }
}
//within 450- 300 . pid to 400  but override with manuaal


void set6MDrive(){
  
  sixMotorDrive = true;
}
void set4MDrive(){

  sixMotorDrive = false;
}

void tiltUp(){
  tilter.set_value(1);
  tilter2.set_value(0);
  TILTUP = true;
}
void tiltDown(){
  tilter.set_value(0);
  tilter2.set_value(1);
  TILTUP = false;
}

void slowTiltDown(){ // REQUIRES A GOAL 
  tilter.set_value(0);
  tilter2.set_value(0);
  TILTUP = false;
}



bool DRIVETRAINHOLD = false;


void climbToggle(){
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
  {
    if(!DRIVETRAINHOLD){
      setBreakTypeHold();
      DRIVETRAINHOLD = true;
    } else{
      setBreakTypeCoast();
      DRIVETRAINHOLD = false;
    }

  }
}


bool ECLAWDOWN = false;
bool fastDropDelay = false; int fastDropCounter = 0;
bool ERELEASETOGGLE = false;
bool slowtiltingnounclamp = false; int noclampcounter = 0;
bool thirdMogoDOwn = false;
bool wpRingValue = false;
void pistons()
{

  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
    wpRing.set_value(!wpRingValue);
    wpRingValue = !wpRingValue;  
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
     if(!thirdMogoDOwn){
       thirdMogo1.set_value(true);
       thirdMogo2.set_value(true);
       thirdMogoDOwn = true;
     }
     else{
       thirdMogo1.set_value(false);
       thirdMogo2.set_value(false);
       thirdMogoDOwn = false;
     }     
  }
  if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
  {

    if (!GRABBED)
    {
      setGrab(true);
    }
    else
    {
      setGrab(false);
    }
  }

  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
    if(CLAMPUP){
      closeClamp();
      tiltToggle = true;
    } else if (!CLAMPUP){
      if(TILTUP && !clamped()){
        tiltDown();
        fastDropDelay = true;
      } else if( TILTUP && clamped()){
        slowTiltDown(); slowTilting = true;

      } else{
        openClamp();
      }
    }
  }

  if(fastDropDelay){
    fastDropCounter++;
    if(fastDropCounter >10){
      fastDropCounter = 0;
      fastDropDelay = false;
      openClamp();
    }
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){

    tiltUp();
  }


  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
     slowTiltDown();    slowtiltingnounclamp = true;

  }

  if(slowtiltingnounclamp){
    noclampcounter++;
    if(noclampcounter > 50){
      tiltDown();
      slowtiltingnounclamp = false;
      noclampcounter = 0;
    }
  }
  if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
    if(ECLAWDOWN){
       eClaw.set_value(0);

    } else{
       eClaw.set_value(1);

    }

    ECLAWDOWN = !ECLAWDOWN;
  }

  if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
    if(grabberDistance.get() < grabberDistanceValue){
      setGrab(true);
    }
  }
  
  // if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
  //   if(ERELEASETOGGLE){
  //      eRelease.set_value(0);
  //   } else{
  //      eRelease.set_value(1);

  //   }

  //   ERELEASETOGGLE = !ERELEASETOGGLE;
  // }
  
  if(tiltToggle){
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      tiltCounter++;
      if(tiltCounter > 30){
        tiltUp();
      }
    } else{
      tiltCounter = 0;
      tiltToggle = 0;
    }
  }

  if(slowTilting){
    counterd++;
    if(counterd > 50){
      tiltDown();
      openClamp();
      slowTilting = false;
      counterd = 0;
    }
  }

}