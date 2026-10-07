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

       static const double PLACE_SETPOINT; 
       static const double PRIMING_SETPOINT; 
       static const double GROUND_SETPOINT; 
       static const double STANDING_SETPOINT;   
   
       //static const double SCOOP_SETPOINT;  
    
       static const double KCOS;

       static Forearm* globalPtr;
        
       double setpoint;
       double startingAngle;

       bool requestingSetpoint = false; 
       double requestedSetpoint; 

       ForearmState currentState = ForearmState::F_HOLDING;  

       vex::motor forearmMotor; 
       vex::rotation rot;
       
       //AngularArmFFConstants armFFConsts; //Bulk (feedforward) 
     
       pidcontroller* feedback = nullptr; //Rest done with feedback
       PIDConstants pidConsts;
        
       double getOutput(); //velocity and acceleration but for angles
       
       double getCurrentAngle(); 
       double getVelocity(); 

       bool safeToManuever(); 

       void maintainHoldLock();  

       void receiveSetpoints(); 
       
       bool underGlobalStall(); 
       
       void setSetpoint(double setp);    
       
       bool reachedSetpoint(); 

       void passMacroTurn(); 
       
       void stateControl();  //ONLY (We can't manually modify the forearm with the controller) 

       void findNextSetpoint();  

       double getError();

    public:   
       using Subsystem::get;  

       static Forearm& getObject();  

       Forearm();

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