#include "vex.h"
#include "hopper.h"
// this function runs once when robot starts

Hopper *Hopper::globalRef = nullptr;

double Hopper::ABSOLUTE_HOPPER_SPEED = 300;
double Hopper::DISTANCE_THRESHOLD = 10; 

void Hopper::init()
{ 
    hopperMotor.setStopping(vex::brakeType::brake);      
}


void Hopper::updateTelemetry()
{
    set<bool>("detects_jam", fabs(getExpectedVelocity() - hopperMotor.velocity(vex::velocityUnits::rpm)) > 1); 
    set<bool>("primed", distanceSensor.objectDistance(vex::distanceUnits::cm) < DISTANCE_THRESHOLD);
}

void Hopper::periodic()
{

    if (RobotState::getStateOf("robot_is_loading") || RobotState::getStateOf("scoring_high") || RobotState::getStateOf("scoring_mid") || RobotState::getStateOf("scoring_low")) // Checks if you should outtake cubes from the hopper
    {
        spinAtPercent(-1);
    }
    else if (RobotState::getStateOf("mixing_hopper"))
    { 
        spinAtPercent(1); 
    }
    else
    { 
        if (!get<bool>("primed") && !(Telemetry::inst.getValueAt<bool>("indexer", "detects_blocks_mid") || Telemetry::inst.getValueAt<bool>("indexer", "detects_blocks_high")) ){ 
          spinAtPercent(-0.05);
        } else { 
          stop();
        }
    }
}

void Hopper::stop()
{
    spinAtPercent(0);
}

void Hopper::spinAtPercent(double percentage){ 
   hopperMotor.setVelocity(ABSOLUTE_HOPPER_SPEED * percentage, vex::velocityUnits::rpm); 
   hopperMotor.spin(vex::directionType::fwd);
}; 


double Hopper::getExpectedVelocity(){ 
    return 0;
};