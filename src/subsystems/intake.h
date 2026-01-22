#ifndef __INTAKE_H__ 
#define __INTAKE_H__ 

#include "../architecture/subsystem.h" 
#include "vex.h" 

class Intake : public Subsystem {  

   private:   
    vex::motor intakeConveyor;

   public:  
     using Subsystem::get;  
     
     static Intake* globalRef;  
     static double ABSOLUTE_CONVEYOR_SPEED;

     Intake(): 
     Subsystem( 
        "intake", 
        { 
            (EntrySet){"is_on", EntryType::BOOL},  
            (EntrySet){"mid_scoring_engaged", EntryType::BOOL},
            (EntrySet){"last_engaged_millis", EntryType::DOUBLE}
        }
     ), 
     intakeConveyor(vex::motor(vex::PORT7))
     { 
       globalRef = this; 
     } 

     void init() override;
     void periodic() override;
     void updateTelemetry() override;
     void stop() override;

   protected:  
     using Subsystem::set; 

}; 

#endif