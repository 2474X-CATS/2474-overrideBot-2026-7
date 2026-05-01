#include "robotConfig.h"

vex::brain Brain;

/*
 TO-DO:
   o- Initialize all vex devices here (motors/sensors/pneumatics/etc)
   o- Initialize constants
*/

//-------
//-------
// Constants: PLACE HERE [regularly]

double ROBOT_WIDTH_MM = 340;  // mm 13in
double ROBOT_LENGTH_MM = 415; // mm 16in

double MOTOR_TEMP_LIMIT_CELSIUS = 55.0;

double TILE_SIZE_MM = (3.6576 / 6) * 1000; // mm

void vexcodeInit() {

};
