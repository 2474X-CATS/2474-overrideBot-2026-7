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

double ROBOT_WIDTH_MM = 15 * 25.4;    // mm 13in
double ROBOT_LENGTH_MM = 12.5 * 25.4; // mm 16in

double MOTOR_TEMP_LIMIT_CELSIUS = 55.0;

double TILE_SIZE_MM = (3.6576 / 6) * 1000; // mm

void vexcodeInit() {

};
