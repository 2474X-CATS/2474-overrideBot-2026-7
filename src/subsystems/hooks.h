#ifndef _HOOKS_H_
#define _HOOKS_H_

#include "../architecture/subsystem.h"
#include "vex.h"


class Hooks : public Subsystem
{
public:
    using Subsystem::get;
    using Subsystem::getFromInputs;
     
    static Hooks* globalRef;

    Hooks() : Subsystem(
                 "descore_hooks",
                 {(EntrySet){"isOn", EntryType::BOOL}} 
                ),  
                hookTank(vex::pneumatics(Brain.ThreeWirePort.A))
                 { 
                    globalRef = this;
                 } 

    void init() override;
    void periodic() override;
    void updateTelemetry() override; 
    void stop() override;  

protected: 
    using Subsystem::set;
private:   
    vex::pneumatics hookTank;  
   

    bool toggled = false;  
    bool holding = false;

    bool toggleSignal();  
};

#endif