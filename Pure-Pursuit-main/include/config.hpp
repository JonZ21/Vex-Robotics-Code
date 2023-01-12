#ifndef CONFIG_HPP
#define CONFIG_HPP
#include "api.h"

//Controllers
extern pros::Controller master;

//Motors
extern pros::Motor leftF; // 6 motor drive
extern pros::Motor leftM;
extern pros::Motor leftB;
extern pros::Motor rightF;
extern pros::Motor rightM;
extern pros::Motor rightB;

extern pros::Motor intake;
extern pros::Motor lift;

//Sensors
extern pros::Distance clampDistance;
extern pros::Distance grabberDistance;
extern pros::Imu imuRight;
extern pros::Imu imuLeft;
extern pros::Rotation leftRotation;
extern pros::Rotation rightRotation;
extern pros::Rotation centerRotation;
extern pros::Rotation liftPot;
extern pros::Vision vision;
extern pros::GPS gpsBack;
extern pros::GPS gpsSide;
    
//2sensorodom
extern pros::Rotation fwdRotation;
extern pros::ADIEncoder sideRotation;

//pistons
extern pros::ADIDigitalOut clamp;
extern pros::ADIDigitalOut tilter;
extern pros::ADIDigitalOut tilter2;

extern pros::ADIDigitalOut thirdMogo1;
extern pros::ADIDigitalOut thirdMogo2;


extern pros::ADIDigitalOut grabber;
extern pros::ADIDigitalOut eRelease;
extern pros::ADIDigitalOut eClaw;

extern pros::ADIDigitalOut wpRing;
#endif
