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
        
       static double GROUND_INTAKE_HEIGHT;  
       static double LEVELED_HEIGHT; 
       
       static double MAX_HEIGHT;

       static double ELEVATOR_ERROR_TOLERANCE; 
       static double STACK_HEIGHT;  

       static double PRIMING_SPEED; 

       static double MINIMUM_ALIGNER_DISTANCE; 
       static double ALIGNER_ERROR_TOLERANCE; 

       static double SPOOL_DIAMETER;

       int raisingDirection;

       ElevatorState currentState = ElevatorState::E_HOLDING;
       
       pidcontroller* correctionController = nullptr; //Adjusting the output of ff so its more accurate
       
       vex::motor lifter1;
       vex::motor lifter2;

       vex::motor_group lift;

       vex::distance primingSensor;

       vex::rotation rot;  

       bool requestingSetpoint = false; 
       double primingSetpoint; 
       double requestedHeight;

       bool reachedSetpoint(); 

       void stateControl();
       void respondToRequests();  
       
       double getPosition();  
       double getVelocity();

       void setSetpoint(double setpoint);   

       void lock(); 

    public:   
       using Subsystem::get; 
       static Elevator& getObject();

       Elevator() : 
       Subsystem( 
          "elevator", 
          { 
            (EntrySet){"active", EntryType::BOOL}, //In a macro?
            (EntrySet){"at_setpoint", EntryType::BOOL}, //Achieved setpoint or no setpoint? 
            (EntrySet){"sensing_stack", EntryType::BOOL},
            (EntrySet){"sniper_score_enabled", EntryType::BOOL},  
            (EntrySet){"percentage_extended", EntryType::DOUBLE}
         }
       ),
       lifter1(vex::motor(vex::PORT14, vex::ratio18_1, true)), 
       lifter2(vex::motor(vex::PORT10, vex::ratio18_1)), 
       lift(vex::motor_group(lifter1, lifter2)), 
       rot(vex::rotation(vex::PORT2)), 
       primingSensor(vex::distance(vex::PORT5))
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


#endif