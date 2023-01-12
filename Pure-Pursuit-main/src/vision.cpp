#include "vision.hpp"
#include "stdlib.h"
#include "initialize.hpp"

// from v5 vision utility
// 1 is the ID

int getX()
{
  // Gets the largest object
  pros::vision_object_s_t rtn = vision.get_by_sig(0, 1);
  return rtn.left_coord;
}
int getY()
{
  // Gets the largest object
  pros::vision_object_s_t rtn = vision.get_by_sig(0, 1);
  return rtn.top_coord;
}

void visionDisplay()
{
  // char buffer[100];
  // sprintf(buffer, "VISION Sensor\n X: %i\nY: %i\n",getX(),getY());
  // lv_label_set_text(infoDisplay,buffer);
}

bool foundObject()
{

  return false;
}
void turnToFaceObject()
{
}

// COLOR CODES CAN BE USED TO TUNE THE SIGNATURE WITH MUTLIPLE COLORS
