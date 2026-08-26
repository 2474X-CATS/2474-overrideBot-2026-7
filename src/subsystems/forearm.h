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

       static Forearm* globalPtr;     

       vex::motor forearmMotor;     

       double startingAngle; 
       double setpoint;

       AngularArmFFConstants armFFConsts; //Bulk (feedforward) 

       pidcontroller* feedback = nullptr; //Rest done with feedback
       PIDConstants pidConsts; 

       TrapezoidConstants motionConsts; 
       TrapezoidalMotionProfile* motionProfile = nullptr;
        
       double angularDeadZones[2];  
       
       double calculateOutput(double omega, double alpha); //velocity and acceleration but for angles
       
       double getCurrentAngle();   
       double getVelocity(); 

       void setSetpoint(double setpoint, bool inverted);    

       int setpointDirection; 

       bool reachedSetpoint();   
       
       ForearmState currentState = ForearmState::HOLDING; 
       
       void updatePosition(); 
       void stateControl();  //ONLY (We can't manually modify the forearm with the controller) 

       void respondToRequests();

    public:   
       using Subsystem::get;

       static Forearm& getObject();  

       Forearm(): 
         Subsystem( 
            "forearm",
            {
               (EntrySet){"at_setpoint", EntryType::BOOL}, 
               (EntrySet){"current_angle", EntryType::DOUBLE},  
               (EntrySet){"current_velocity", EntryType::DOUBLE},
               (EntrySet){"requesting_setpoint", EntryType::BOOL}, 
               (EntrySet){"requested_angle", EntryType::DOUBLE}, 
               (EntrySet){"requested_velocity", EntryType::DOUBLE}
            }
         ),
         forearmMotor(vex::motor(vex::PORT4, vex::ratio18_1))
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