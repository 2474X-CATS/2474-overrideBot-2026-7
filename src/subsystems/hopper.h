#ifndef __HOPPER_H__
#define __HOPPER_H__

#include "../architecture/subsystem.h"

class Hopper : public Subsystem
{
public:
    using Subsystem ::get;
    
    static Hopper* globalRef; 

    Hopper() : Subsystem(
                   "hopper",
                   {(EntrySet){"isOn", EntryType::BOOL}} 
               ), 
               hopperMotor(vex::motor(vex::PORT18, false)) 
               //containerSensor(vex::distance(vex::PORT22))
    { 
        globalRef = this;
    }
   
    void init() override;           
    void periodic() override;       
    void updateTelemetry() override; 
    void stop() override;

    void dispenseCubes(); 
    void mixHopper();  

private:   
    vex::motor hopperMotor; 
    //vex::distance containerSensor;  

    bool shouldDispenseCubes();  
    bool shouldMixHopper();  

    // this figures out what the driver wants the hopper to do
   
};

#endif
