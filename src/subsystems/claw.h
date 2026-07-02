#ifndef __CLAW_H__ 
#define __CLAW_H__ 

#include "../architecture/subsystem.h"
#include "vex.h" 

class Claw : public Subsystem {  
    
    private:  
       static Claw* globalPtr; 

       vex::pneumatics wrist; 
       vex::pneumatics claw;

       void flip(bool facingDown);  
       void clench(bool clenched); 

       bool canGrab();

    public:   
       using Subsystem::get; 
       static Claw& getObject();
       
       Claw() : 
       Subsystem( 
        "claw", 
        { 
           (EntrySet){"clenched", EntryType::BOOL}, 
           (EntrySet){"facing_down", EntryType::BOOL}, 
           (EntrySet){"active", EntryType::BOOL}, 
        } 
        ), 
       claw(vex::pneumatics(Brain.ThreeWirePort.B)), 
       wrist(vex::pneumatics(Brain.ThreeWirePort.E)) 
       { 
        globalPtr = this;
       };
       
       void init() override; 
       void periodic() override; 
       void updateTelemetry() override; 
       void stop() override;

       void respondToRequests(); 

       void stateControl();   
       
    protected: 
       using Subsystem::set; 

};

#endif 