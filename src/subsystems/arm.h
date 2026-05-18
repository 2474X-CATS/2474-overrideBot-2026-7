#ifndef __ARM_H__ 
#define __ARM_H__ 

#include "../architecture/subsystem.h" 


class Arm : public Subsystem {  

    private: 
      vex::motor leftMotor; 
      vex::motor rightMotor; 

    protected: 
      using Subsystem::set;   

    public: 
      using Subsystem::get;  

      Arm() : 
      Subsystem( 
        "arm", 
        { 
          {"is_on", EntryType::BOOL},    
        }
      ), 
      leftMotor(vex::motor(vex::PORT19)), 
      rightMotor(vex::motor(vex::PORT20)){}; 

      void init() override; 
      void periodic() override; 
      void updateTelemetry() override;  
      void stop() override;

};

#endif 
