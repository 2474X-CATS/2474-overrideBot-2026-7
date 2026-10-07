#ifndef __ELEVATOR_H__ 
#define __ELEVATOR_H__ 

#include "../architecture/subsystem.h"  
#include "../architecture/command.h"  

#include "../control/feedForward.h" 
#include "../control/pidcontroller.h" 
#include "../control/trapezoidalMotion.h"


typedef enum { 
   E_HOLDING, 
   E_PRIMING, 
   E_PURSUING, 
   E_ADJUSTING, 
   E_LISTLESS
} ElevatorState;

class Elevator : public Subsystem { 
    
    private:   
       
       static Elevator* globalPtr;  
      
       static const double GROUND_PRESSURE; 

       static const double GROUND_INTAKE_HEIGHT;   
       static const double PRIMING_HEIGHT;
       
       static const double MAX_HEIGHT;
   
       static const double PRIMING_SPEED; 

       static const double MINIMUM_ALIGNER_DISTANCE; 
       //static const double ALIGNER_ERROR_TOLERANCE; 

       static const double SPOOL_DIAMETER;

       int raisingDirection = 0;

       ElevatorState currentState = ElevatorState::E_HOLDING;
       
       pidcontroller* correctionController = nullptr; //Adjusting the output of ff so its more accurate
       
       vex::motor lifter1;
       vex::motor lifter2;
       vex::motor_group lift;
       vex::distance primingSensor;
       vex::rotation rot;  

       double primingSetpoint = 0; 
       bool reachedSetpoint();  

       bool underGlobalStall(); 

       void stateControl();
       void respondToRequests();   

       void receiveSetpoints();  

       bool safeToManuever(); 
       void maintainHoldLock();  

       void findNextSetpoint(); 
       void regulatePriming();
       
       double getPosition();  
       double getVelocity();

       void setSetpoint(double setpoint);   

       void lock(); 

    public:    
       using Subsystem::get; 
       static Elevator& getObject(); 

       static const double LEVELED_HEIGHT;

       Elevator();
       
       void init() override; 
       void periodic() override;
       void updateTelemetry() override; 
       void stop() override;  
      
    protected: 
       using Subsystem::set; 

}; 

class RunElevator : public Command<Elevator> { 
  private: 
   Elevator& elevatorRef; 

  public:   

   static CommandInterface* getCommand(){ 
      return new RunElevator(Elevator::getObject()); 
   } 

   RunElevator(Elevator& elevator) :   
   Command<Elevator>(elevator),
   elevatorRef(elevator) 
   {};

  protected:
   void start() override; 
   void periodic() override; 
   bool isOver() override; 
   void end() override;
};  

//---------------------------------------------------------------------



#endif