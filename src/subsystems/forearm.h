#ifndef __FOREARM_H__ 
#define __FOREARM_H__ 

#include "../architecture/subsystem.h"   
//#include "../control/feedForward.h" 
//#include "../control/pidcontroller.h"   
#include "../control/trapezoidalMotion.h"

typedef enum { 
   F_PURSUING,  
   F_HOLDING
} ForearmState;

class Forearm : public Subsystem {  
    private:

       static double PLACE_SETPOINT; 
       static double PRIMING_SETPOINT; 
       static double GROUND_SETPOINT; 
       static double STANDING_SETPOINT;   
       static double RELEASE_SETPOINT;

       static double ANGULAR_ERROR_TOLERANCE; 

       static Forearm* globalPtr;     

       TrapezoidConstants motionConsts; 
       TrapezoidalMotionProfile* motionProfile = nullptr;
        
       double angularDeadZones[2];  
       
       double getCurrentAngle();   
       double getVelocity(); 

       double previousAngle; 
       double previousTimestamp; 

       bool requestingSetpoint = false; 
       double requestedSetpoint;

       void setSetpoint(double setpoint, bool inverted);    

       int setpointDirection; 

       bool reachedSetpoint();   

       ForearmState currentState = ForearmState::F_HOLDING; 
       
       void updatePosition(double velocity); 
       void stateControl();  //ONLY (We can't manually modify the forearm with the controller)

    public:   
       using Subsystem::get;  

       static Forearm& getObject();  

       Forearm(): 
         Subsystem( 
            "forearm", 
            {  
               (EntrySet){"task_id", EntryType::INT}, 
               (EntrySet){"active", EntryType::BOOL}, 
               (EntrySet){"at_setpoint", EntryType::BOOL}, 
               (EntrySet){"current_angle", EntryType::DOUBLE}, 
               (EntrySet){"hold", EntryType::BOOL}
            }
         )
         { 
            globalPtr = this;
         };    

         void init() override; 
         void periodic() override; 
         void updateTelemetry() override; 
         void stop() override;  
      

    protected: 
         using Subsystem::set; 
    
      
}; 


  

#endif