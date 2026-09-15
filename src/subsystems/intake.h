#ifndef __INTAKE_H__ 
#define __INTAKE_H__ 

#include "../architecture/subsystem.h"


class Intake : public Subsystem { 

    private:   
      vex::motor intakeMotor;  
      
    public:
      Intake(): 
      Subsystem( 
        "intake", 
        { 
          (EntrySet){"intaking", EntryType::BOOL}, 
          (EntrySet){"outtaking", EntryType::BOOL}
        }
      ),  
      intakeMotor(vex::motor(vex::PORT9))
      {}; 
      
      void init() override; 
      void periodic() override; 
      void updateTelemetry() override; 
      void stop() override;
};


#endif 