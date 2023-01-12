#include "config.hpp"
#include "main.h"
#include "api.h"

// Controllers
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Motors
pros::Motor leftF(17, pros::E_MOTOR_GEARSET_06); // 6 motor drive
pros::Motor leftM(16, pros::E_MOTOR_GEARSET_06);
pros::Motor leftB(11, pros::E_MOTOR_GEARSET_18);
pros::Motor rightF(12, pros::E_MOTOR_GEARSET_18);
pros::Motor rightM(7, pros::E_MOTOR_GEARSET_18);
pros::Motor rightB(5, pros::E_MOTOR_GEARSET_18);
pros::Motor lift(18, pros::E_MOTOR_GEARSET_36);
pros::Motor intake(14, pros::E_MOTOR_GEARSET_06);

// Sensors
pros::Distance clampDistance(3);
pros::Distance grabberDistance(1);
pros::Imu imuRight(8);
pros::Imu imuLeft(15);
pros::Rotation leftRotation(0);
pros::Rotation rightRotation(0);
pros::Rotation fwdRotation(13);
pros::ADIEncoder sideRotation('E','F');
pros::Rotation centerRotation(0);
pros::Rotation liftPot(6);
pros::Vision vision(0);
pros::GPS gpsBack(0);
pros::GPS gpsSide(0);

// Pistons
pros::ADIDigitalOut clamp('H');
pros::ADIDigitalOut tilter('D');
pros::ADIDigitalOut tilter2('G');
pros::ADIDigitalOut grabber('A');

//3rd mogo
pros::ADIDigitalOut thirdMogo1({{9,'F'}});
pros::ADIDigitalOut thirdMogo2({{9,'G'}});

//erector
pros::ADIDigitalOut eRelease('C'); 
pros::ADIDigitalOut eClaw({{9,'H'}}); 


// wp
pros::ADIDigitalOut wpRing('B');
