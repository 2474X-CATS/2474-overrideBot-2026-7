#ifndef __MATCHLOADER_H__
#define __MATCHLOADER_H__


#include "../architecture/subsystem.h"


class Matchloader : public Subsystem {
public:
    using Subsystem::get;
    
    static Matchloader* globalRef;  

    Matchloader() : Subsystem(
        "matchloader",
        {(EntrySet){"isOn", EntryType::BOOL}}
    ), 
    matchloaderPiston(vex::pneumatics(Brain.ThreeWirePort.H))  
    { 
        globalRef = this;
    };

    void init() override;
    void periodic() override;
    void updateTelemetry() override;
    void stop() override;  

    void deploy(); 
    void retract();
protected:
    using Subsystem::set; 
private:  
    vex::pneumatics matchloaderPiston;  
};

#endif