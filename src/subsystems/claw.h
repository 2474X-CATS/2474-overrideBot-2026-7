#ifndef __CLAW_H__ 
#define __CLAW_H__ 

#include "../architecture/subsystem.h"
#include "vex.h" 

class Claw : public Subsystem {  
    
    private:  
       static Claw* globalPtr;  
       static double MAXIMUM_TOLERABLE_DISTANCE;
       
       /*
       vex::pneumatics wrist; 
       vex::pneumatics claw; 

       vex::distance objectDetector;
       */ 

       void flip(bool facingDown);  
       void clench(bool clenched); 

       bool sensesObject();

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
           (EntrySet){"requesting_act", EntryType::BOOL}, //Does the next action want to be taken depending on the state
           (EntrySet){"senses_object", EntryType::BOOL}
        } 
        )
       //claw(vex::pneumatics(Brain.ThreeWirePort.B)), 
       //wrist(vex::pneumatics(Brain.ThreeWirePort.E)), 
       //objectDetector(vex::distance(vex::PORT13))
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