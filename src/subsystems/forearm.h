#ifndef __FOREARM_H__ 
#define __FOREARM_H__ 

#include "../architecture/subsystem.h"   
#include "../architecture/command.h"  

#include "../control/feedForward.h" 
#include "../control/pidcontroller.h"   
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

       static Forearm* globalPtr;
        
       double setpoint;
       double startingAngle;

       bool requestingSetpoint = false; 
       double requestedSetpoint;

       double angularDeadZones[2];   
       
       ForearmState currentState = ForearmState::F_HOLDING;  

       vex::motor forearmMotor;
       
       AngularArmFFConstants armFFConsts; //Bulk (feedforward) 
     
       pidcontroller* feedback = nullptr; //Rest done with feedback
       PIDConstants pidConsts;

       TrapezoidConstants motionConsts;
       TrapezoidalMotionProfile* motionProfile = nullptr;
        
       double calculateOutput(double omega, double alpha); //velocity and acceleration but for angles
       
       double getCurrentAngle(); 
       double getVelocity();

       void setSetpoint(double setp, bool inverted);    
       
       bool reachedSetpoint();
       
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
               //(EntrySet){"current_angle", EntryType::DOUBLE} 
            }
         ),
         forearmMotor(vex::motor(vex::PORT4))
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

class RunForearm : public Command<Forearm> { 
  private: 
   Forearm& forearmRef; 

  public:   

   static CommandInterface* getCommand(){ 
      return new RunForearm(Forearm::getObject()); 
   } 

   RunForearm(Forearm& forearm) :   
   Command<Forearm>(forearm),
   forearmRef(forearm)
   {};   

  protected: 
   void start() override; 
   void periodic() override; 
   bool isOver() override; 
   void end() override;
};

  

#endif