#ifndef __DRIVEBASE_H__ 
#define __DRIVEBASE_H__ 


#include "../architecture/subsystem.h"  
#include "../control/feedForward.h" 
#include "../control/pidcontroller.h" 
#include "../control/trapezoidalMotion.h"

class Drivebase : public Subsystem { 
    public:  
      using Subsystem::get;   
      
      static Drivebase& getObject(); 

      Drivebase(): 
      Subsystem( 
        "drivebase", 
        { 
          (EntrySet){"is_on", EntryType::BOOL}
        }
      ), 
      leftFront(vex::motor(vex::PORT7)), 
      leftBack(vex::motor(vex::PORT6)), 
      rightFront(vex::motor(vex::PORT5)), 
      rightBack(vex::motor(vex::PORT4)), 
      leftMotors(leftFront, leftBack), 
      rightMotors(rightFront, rightBack)
      { 
        globalPtr = this;
      };
      
      void manualDrive(double voltage); 
      
      void manualSpin(double voltage); //Negative for the opposite direction


    private:     
      static Drivebase* globalPtr;

      vex::motor leftFront; 
      vex::motor leftBack; 

      vex::motor rightFront; 
      vex::motor rightBack;  

      vex::motor_group leftMotors;
      vex::motor_group rightMotors;

      FFConstants ffConsts;
      PIDConstants correctionConsts; 
      TrapezoidConstants motionConsts;
      
      void arcadeDrive(double speed, double rotation); 

    protected: 
      using Subsystem::set; 

      void init() override;
      void periodic() override; 
      void updateTelemetry() override; 
      void stop() override; 

};


#endif