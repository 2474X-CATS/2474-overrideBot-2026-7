#ifndef _INTAKE_H_
#define _INTAKE_H_
#include "../architecture/subsystem.h"

#include "vex.h"


class Intake : public Subsystem {
public: 
    using Subsystem::get;  
    using Subsystem::getFromInputs; 

    static Intake* globalRef; 

    Intake() : Subsystem(
                    "intake",
                    {(EntrySet){"isOn", EntryType::BOOL}
                    }), 
                    intakeMotor(vex::motor(vex::PORT17, false))
                    { 
                        globalRef = this;
                    }  
    void init() override; 
    void periodic() override; 
    void updateTelemetry() override;
    void stop() override; 

    void intake(); 
    void outtake();  

private:   
    static double ABSOLUTE_INTAKE_SPEED; 
    vex::motor intakeMotor;  

    //vex::distance channelSensor; 

    bool shouldIntake(); 
    bool shouldOuttake();   
    
};

#endif
