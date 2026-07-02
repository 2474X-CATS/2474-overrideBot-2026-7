#ifndef __DRIVEBASE_H__ 
#define __DRIVEBASE_H__ 


#include "../architecture/subsystem.h"  
#include "../control/feedForward.h"

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
      ){ 
        globalPtr = this;
      };
      
      void manualDrive(double speed, double acceleration); 
      
      void manualSpin(double voltage); //Negative for the opposite direction

    private:     
      static Drivebase* globalPtr;

      vex::motor leftFront; 
      vex::motor leftBack; 

      vex::motor rightFront; 
      vex::motor rightBack;    

      FFConstants ffConsts;   

      void arcadeDrive(double speed, double rotation); 

    protected: 
      using Subsystem::set; 

      void init() override;
      void periodic() override; 
      void updateTelemetry() override; 
      void stop() override; 

}


#endif