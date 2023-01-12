#ifndef INITIALIZE_HPP
#define INITIALIZE_HPP

#include "main.h"
#include "config.hpp"
#include "odom.hpp"
#include "robot.hpp"
#include "movement.hpp"
#include "PID.hpp"
#include "api.h"
#include "auton.hpp"
#include "pros/apix.h"
#include "display/lvgl.h"

extern lv_obj_t * OdomDisplay;
extern lv_obj_t * AutonDisplay;
extern lv_obj_t * infoDisplay;
extern lv_obj_t * infoDisplay2;
extern lv_obj_t * mainDisplay;
extern int autonChoice;

#endif
