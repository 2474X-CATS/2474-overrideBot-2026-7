#ifndef __WEDGE_H__ 
#define __WEDGE_H__ 

#include "../architecture/subsystem.h" 
#include "vex.h" 

class Wedge : public Subsystem {  

   private:   
    vex::pneumatics wedgePiston1; 
    vex::pneumatics wedgePiston2; 
    vex::distance wedgeSensor;  

    vex::pneumatics wedgeCloser; 

   public:  
     using Subsystem::get;  
     
     static Wedge* globalRef; 

     Wedge(): 
     Subsystem( 
        "wedge", 
        { 
            (EntrySet){"is_on", EntryType::BOOL}
        }
     ), 
     wedgePiston1(vex::pneumatics(Brain.ThreeWirePort.A)), 
     wedgePiston2(vex::pneumatics(Brain.ThreeWirePort.D)),  
     wedgeSensor(vex::distance(vex::PORT19)),  
     wedgeCloser(vex::pneumatics(Brain.ThreeWirePort.G))
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