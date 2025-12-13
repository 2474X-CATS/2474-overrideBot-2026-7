#ifndef _HOOD_H_
#define _HOOD_H_

#include "../architecture/subsystem.h"
#include "vex.h"

class Hood : public Subsystem
{
public:
    using Subsystem::get;

    static Hood *globalRef;

    Hood() : Subsystem(
                 "hood",
                 { 
                    (EntrySet){"isOn", EntryType::BOOL}, 
                    (EntrySet){"timestampHigh", EntryType::DOUBLE} 
                }),
             hoodPiston(vex::pneumatics(Brain.ThreeWirePort.G)), 
             highPiston(vex::pneumatics(Brain.ThreeWirePort.C))
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

    vex::pneumatics hoodPiston;
    vex::pneumatics highPiston; 

    void open();
    void close();

    bool holding = false;

    bool shouldOpen();
    bool shouldClose();
    bool isHolding();
};

#endif