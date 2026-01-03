#ifndef __INDEXER_H__ 
#define __INDEXER_H__ 

#include "../architecture/subsystem.h" 
#include "vex.h" 

class Indexer : public Subsystem {  

   private:   
    vex::motor indexerMotor;  
    vex::pneumatics indexerHatch;  

   public:  
     using Subsystem::get;  
     
     static Indexer* globalRef;  
     static double ABSOLUTE_INDEXER_SPEED;

     Indexer(): 
     Subsystem( 
        "indexer", 
        { 
            (EntrySet){"is_on", EntryType::BOOL}
        }
     ), 
     indexerMotor(vex::motor(vex::PORT8)), 
     indexerHatch(vex::pneumatics(Brain.ThreeWirePort.F))
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