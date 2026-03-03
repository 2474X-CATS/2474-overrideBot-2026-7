#ifndef __CATAPULT_H__ 
#define __CATAPULT_H__ 

#include "../architecture/subsystem.h" 
#include "vex.h" 

class Catapult : public Subsystem {  

   private:   
    
   public:  
     using Subsystem::get;  
     
     static Catapult* globalRef; 

     Catapult(): 
     Subsystem( 
        "catapult", 
        { 
            (EntrySet){"is_on", EntryType::BOOL}
        }
     )
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