#ifndef __HOPPER_H__
#define __HOPPER_H__

#include "../architecture/subsystem.h"


class Hopper : public Subsystem
{
public:
    using Subsystem ::get;

    static Hopper *globalRef;

    Hopper() : Subsystem(
                   "hopper",
                   {  
                    (EntrySet){"detects_jam", EntryType::BOOL}
                   }),
               hopperMotor(vex::motor(vex::PORT18, false))
    {
        globalRef = this;
    }

    void init() override;
    void periodic() override;
    void updateTelemetry() override;
    void stop() override;

private:
    vex::motor hopperMotor;
    
    static double ABSOLUTE_HOPPER_SPEED;

    bool shouldDispenseCubes();
    bool shouldMixHopper();

    void dispenseCubes();
    void mixHopper(); 

    double getExpectedVelocity(); 
    
};

#endif
