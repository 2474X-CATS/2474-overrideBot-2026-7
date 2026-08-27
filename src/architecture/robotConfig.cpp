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

double ROBOT_WIDTH_MM = 13 * 25.4;  // mm 13in
double ROBOT_LENGTH_MM = 15 * 25.4; // mm 16in

double MOTOR_TEMP_LIMIT_CELSIUS = 55.0;

double TILE_SIZE_MM = (12 * 12 * 25.4) / 6; //mm 12 feet by 12 feet field

void vexcodeInit() {

};
