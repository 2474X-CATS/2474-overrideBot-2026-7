#ifndef __HOOKS_H__ 
#define __HOOKS_H__ 

#include "../architecture/subsystem.h" 
#include "vex.h" 

class Hooks : public Subsystem {  

   private:   
    vex::pneumatics hookPiston;

   public:  
     using Subsystem::get;  
     
     static Hooks* globalRef; 

     Hooks(): 
     Subsystem( 
        "hooks", 
        { 
            (EntrySet){"is_on", EntryType::BOOL}
        }
     ), 
     hookPiston(vex::pneumatics(Brain.ThreeWirePort.C)) 
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