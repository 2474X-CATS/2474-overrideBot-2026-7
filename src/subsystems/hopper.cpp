#include "vex.h"
#include "hopper.h"
// this function runs once when robot starts

Hopper *Hopper::globalRef = nullptr;

double Hopper::ABSOLUTE_HOPPER_SPEED = 300;

void Hopper::init()
{
    hopperMotor.setStopping(vex::brakeType::brake);      
}


void Hopper::updateTelemetry()
{
    set<bool>("detects_jam", fabs(getExpectedVelocity() - hopperMotor.velocity(vex::velocityUnits::rpm)) > 1);
}

void Hopper::periodic()
{

    if (shouldDispenseCubes()) // Checks if you should outtake cubes from the hopper
    {
        dispenseCubes();
    }
    else if (shouldMixHopper())
    { // Checks if you should run hopper motor away from outtake
        mixHopper();
    }
    else
    {
        stop(); // Checks if you should stop the hopper motor
    }
}

void Hopper::dispenseCubes()
{
    hopperMotor.setVelocity(ABSOLUTE_HOPPER_SPEED, vex::velocityUnits::rpm);
    hopperMotor.spin(vex::directionType::fwd);
}

void Hopper::mixHopper()
{
    hopperMotor.setVelocity(-ABSOLUTE_HOPPER_SPEED, vex::velocityUnits::rpm);
    hopperMotor.spin(vex::directionType::fwd);
}

void Hopper::stop()
{
    hopperMotor.setVelocity(0, vex::percentUnits::pct);
    hopperMotor.spin(vex::directionType::fwd);
}

bool Hopper::shouldDispenseCubes()
{
    return RobotState::getStateOf("scoring_high") || RobotState::getStateOf("scoring_mid") || RobotState::getStateOf("scoring_low");
}

bool Hopper::shouldMixHopper()
{
    return RobotState::getStateOf("mixing_hopper");
}

double Hopper::getExpectedVelocity(){ 
    double output; 
    if (shouldDispenseCubes()) 
      output = ABSOLUTE_HOPPER_SPEED; 
    else if (shouldMixHopper()) 
      output = -ABSOLUTE_HOPPER_SPEED; 
    else { 
      output = 0;
    } 
    return output; 
};