#include "intake.h"
#include "vex.h"

Intake *Intake::globalRef = nullptr;

double Intake::ABSOLUTE_INTAKE_SPEED = 200;

void Intake::init()
{
    set<bool>("isOn", true);
};

void Intake::periodic()
{ 
    if (RobotState::getStateOf("robot_is_loading")){ 
        spinAtPercent(-0.1);
    } 
    else if (RobotState::getStateOf("scoring_high") || RobotState::getStateOf("scoring_mid"))
    {
        spinAtPercent(-1); //Upwards
    } 
    else if ( RobotState::getStateOf("intaking_to_hopper")){  
        if (RobotState::getStateOf("color_sensitive"))
          spinAtPercent(-0.6); 
        else { 
          spinAtPercent(-1); 
        }
    }
    else if (RobotState::getStateOf("scoring_low") || RobotState::getStateOf("reverse_intake"))
    {
        spinAtPercent(1); //Downwards
    }
    else
    {
        stop();
    }
};

void Intake::updateTelemetry()
{
    return;
}


void Intake::spinAtPercent(double percentage){ 
    intakeMotor.setVelocity(percentage * ABSOLUTE_INTAKE_SPEED, vex::velocityUnits::rpm); 
    intakeMotor.spin(vex::directionType::fwd);
} 


void Intake::stop()
{
    spinAtPercent(0);
}
