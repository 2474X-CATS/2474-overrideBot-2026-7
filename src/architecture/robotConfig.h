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

typedef enum {
    //INTAKING = 1, //Elevator dropped and the claw ready to receive (no auto transition), can intake
    PRIMED = 1, // Elevator active ,forearm out, has something, ready to score
    MATCHLOADING,
    AUTO // Scoring macro activated 
} SuperStructurePosition;

//-------

void vexcodeInit();
