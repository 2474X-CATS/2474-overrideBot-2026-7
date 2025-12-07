#pragma once

#include "vex.h"

extern vex::brain Brain;

/*
 TO-DO:
   o- Initialize all vex devices here (motors/sensors/pneumatics/etc)
   o- Initialize constants
*/

//-------

// Constants: PLACE HERE [using extern]

extern double ROBOT_WIDTH_MM;
extern double ROBOT_LENGTH_MM; 

extern double MOTOR_TEMP_LIMIT_CELSIUS;

extern double TILE_SIZE_MM;

//-------

void vexcodeInit();
