#ifndef __ELEVATOR_H__ 
#define __ELEVATOR_H__ 

#include "../architecture/subsystem.h"   

#include "../control/feedForward.h" 
#include "../control/pidcontroller.h" 
#include "../control/trapezoidalMotion.h"


typedef enum { 
   HOLDING, 
   PRIMING, 
   PURSUING, 
   ADJUSTING
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

       ElevatorState currentState = ElevatorState::HOLDING; 
       
       int raisingDirection; 
       
       pidcontroller* correctionController = nullptr; 
       
       bool reachedSetpoint(); 

       void stateControl();
       void respondToRequests();  
       
       double getPosition();  
       double getVelocity();

       void setSetpoint(double setpoint);   
       void settle(); 


       vex::motor lifter1; 
       vex::motor lifter2;  

       vex::motor_group lift;  

       vex::distance primingSensor;

       vex::rotation rot;  

    public:   
       using Subsystem::get; 
       static Elevator& getObject();

       Elevator() : 
       Subsystem( 
          "elevator", 
          { 
            (EntrySet){"active", EntryType::BOOL}, //In a macro?
            (EntrySet){"at_setpoint", EntryType::BOOL}, //Achieved setpoint or no setpoint? 
            (EntrySet){"requesting_setpoint", EntryType::BOOL},
            (EntrySet){"requested_height", EntryType::DOUBLE}, //Specifically what height do we want to reach 
            (EntrySet){"has_setpoint", EntryType::BOOL}, //Do we have a specific height we want to reach 
            (EntrySet){"sensing_stack", EntryType::BOOL}, 
            (EntrySet){"current_height", EntryType::DOUBLE}, 
            (EntrySet){"current_velocity", EntryType::DOUBLE}, 
         }
       ),
       lifter1(vex::motor(vex::PORT14, vex::ratio18_1, true)), 
       lifter2(vex::motor(vex::PORT10, vex::ratio18_1)), 
       lift(vex::motor_group(lifter1, lifter2)), 
       rot(vex::rotation(vex::PORT2)), //Need correct port
       primingSensor(vex::distance(vex::PORT20))
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