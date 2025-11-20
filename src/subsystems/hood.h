#ifndef _HOOD_H_
#define _HOOD_H_

#include "../architecture/subsystem.h"
#include "vex.h"


class Hood : public Subsystem
{
public:
    using Subsystem::get;
     
    static Hood* globalRef;

    Hood() : Subsystem(
                 "hood",
                 {(EntrySet){"isOn", EntryType::BOOL}}), 
                 hoodPiston(vex::pneumatics(Brain.ThreeWirePort.G))  
                 { 
                    globalRef = this;
                 } 

    void init() override;
    void periodic() override;
    void updateTelemetry() override; 
    void stop() override;  

    void open(); 
    void close();

protected: 
    using Subsystem::set;
private:   
    vex::pneumatics hoodPiston; 

    bool holding = false;  

    bool shouldOpen(); 
    bool shouldClose();  
    bool isHolding(); 
};

#endif