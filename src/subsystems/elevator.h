#ifndef __ELEVATOR_H__ 
#define __ELEVATOR_H__ 

#include "../architecture/subsystem.h"  
#include "../architecture/command.h"  

#include "../control/feedForward.h" 
#include "../control/pidcontroller.h" 
#include "../control/trapezoidalMotion.h"


typedef enum { 
   E_HOLDING, 
   E_SCORING, 
   E_PURSUING
} ElevatorState;

class Elevator : public Subsystem { 
    
    private:
       
       static Elevator* globalPtr;  
      
       static const double MAX_HEIGHT;
       static const double SPOOL_DIAMETER;
       static const double PRIMING_SPEED;
       static const int MACRO_DURATION; 

       int raisingDirection = 0;
       int scoreStamp = 0;

       double currentSetpoint;
       
       void updateDrivercontrol(); 
       void updateAuto();  

       ElevatorState currentState = ElevatorState::E_HOLDING;
       
       vex::motor lifter1;
       vex::motor lifter2; 
       vex::motor lifter3; 
       
       vex::motor_group lift; 
       vex::rotation rot;

       bool reachedSetpoint();    

       void findNextSetpoint();
       void regulatePriming();
       
       double getPosition();

       void setSetpoint(double setpoint);   

    public:    
       using Subsystem::get; 
       static Elevator& getObject(); 

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


class FrontRunElevatorSetpoint : public Command<DummySystem>{  
   private:  
     static const double STACK_HEIGHT; //Height of half a pin
     
     int stacks;   
     double offset;  

     bool ran = false;
   public:

     static CommandInterface* getCommand(double stackHeight, double goalOffset){ 
      return new FrontRunElevatorSetpoint(stackHeight, goalOffset); 
     } 

     FrontRunElevatorSetpoint(double stackHeight, double goalOffset):
     Command<DummySystem>(GLOBAL_DUMMY), 
     stacks(stackHeight),  
     offset(goalOffset)
     {};   

   protected:  
     void start() override; 
     void periodic() override; 
     bool isOver() override; 
     void end() override;
};

//---------------------------------------------------------------------



#endif