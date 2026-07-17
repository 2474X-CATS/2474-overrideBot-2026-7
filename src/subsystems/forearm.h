#ifndef __FOREARM_H__ 
#define __FOREARM_H__ 

#include "../architecture/subsystem.h"   
#include "../control/feedForward.h" 
#include "../control/pidcontroller.h"   
#include "../control/trapezoidalMotion.h"

typedef enum { 
   PURSUING,  
   HOLDING
} ForearmState;

class Forearm : public Subsystem {  
    private:

       static double PLACE_SETPOINT; 
       static double PRIMING_SETPOINT; 
       static double GROUND_SETPOINT; 
       static double STANDING_SETPOINT;  

       static double ANGULAR_ERROR_TOLERANCE; 

       static Forearm* globalPtr;     

       double currentAngle; 
       double currentVelocity; 

       TrapezoidConstants motionConsts; 
       TrapezoidalMotionProfile* motionProfile = nullptr;
        
       double angularDeadZones[2];  
       
       double getCurrentAngle();   
       double getVelocity(); 

       double previousAngle; 
       double previousTimestamp;

       void setSetpoint(double setpoint, bool inverted);    

       int setpointDirection; 

       bool reachedSetpoint();  

       double angularSetpoint; 

       ForearmState currentState = ForearmState::HOLDING; 
       
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
               (EntrySet){"at_setpoint", EntryType::BOOL} 
               (EntrySet){"current_angle", EntryType::DOUBLE}
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