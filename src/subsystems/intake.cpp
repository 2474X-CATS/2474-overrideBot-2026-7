#include "intake.h"
#include "vex.h"


Intake* Intake::globalRef = nullptr; 

double Intake::ABSOLUTE_INTAKE_SPEED = 200;  

void Intake::init()
{  
    set<bool>("isOn", true); 
};

void Intake::periodic()
{ 
    if (shouldIntake())
    {  
        intake();
    }
    else if (shouldOuttake())
    { 
        intake();
    }
    else
    { 
        stop();
    } 


    
};

void Intake::updateTelemetry()
{      
    return;
   //Telemetry::inst.placeValueAt<double>(intakeMotor.temperature(), "Motor_Temps","IntakeMotor");
}

bool Intake::shouldIntake()
{
    return getFromInputs<bool>("Controller/Button_R1") ||
           getFromInputs<bool>("Controller/Button_R2") || 
           getFromInputs<bool>("Controller/Button_Y");
}

bool Intake::shouldOuttake()
{
    return getFromInputs<bool>("Controller/Button_B") ||  
           getFromInputs<bool>("Controller/Button_RIGHT");
} 

void Intake::intake(){  
    intakeMotor.setVelocity(-ABSOLUTE_INTAKE_SPEED, vex::velocityUnits::rpm); 
    intakeMotor.spin(vex::directionType::fwd);
}  

void Intake::outtake(){ 
    intakeMotor.setVelocity(ABSOLUTE_INTAKE_SPEED * 0.5, vex::velocityUnits::rpm); 
    intakeMotor.spin(vex::directionType::fwd);
}  

void Intake::stop(){ 
    intakeMotor.setVelocity(0, vex::percentUnits::pct); 
    intakeMotor.spin(vex::directionType::fwd);
} 


