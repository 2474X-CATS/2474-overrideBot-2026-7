#ifndef __ELEVATOR_H__ 
#define __ELEVATOR_H__ 

#include "../architecture/subsystem.h"   

#include "../control/feedForward.h" 
#include "../control/pidcontroller.h" 
#include "../control/trapezoidalMotion.h"


typedef enum { 
   HOLDING, 
   PRIMING, 
   PURSUING
} ElevatorState;

class Elevator : public Subsystem { 
    
    private:   
       
       static Elevator* globalPtr = nullptr;  
        
       static double GROUND_INTAKE_HEIGHT;  
       static double LEVELED_HEIGHT; 

       static double ELEVATOR_ERROR_TOLERANCE; 
       static double STACK_HEIGHT;  

       static double PRIMING_SPEED;

       ElevatorState currentState; 

       double currentVelocity;   

       int raisingDirection = 0;
       
       double currentHeight;  

       double currentSetpoint;
       
       bool initialPrimingState; 
       int setpointDirection;


       TrapezoidalMotionProfile* motionProfile = nullptr; //When the robot has defined setpoints it needs to reach
       TrapezoidConstants* motionConsts = nullptr;
       errorcontroller* correctionController = nullptr; //Adjusting the output of ff so its more accurate
       ElevatorFFConstants* elevatorFF = nullptr; //Elevator FF Running at almost all times
       
       double calculateOutput(double velocity, double acceleration); 
       
       void stateControl();
       void respondToRequests();  

       void setSetpoint(double setpoint); 

       vex::motor lifter1; 
       vex::motor lifter2;  

       vex::motor_group lift;

       vex::rotation rot;  

    public:   

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
            (EntrySet){"sensing_stack", EntryType::BOOL}
         }
       ),
       lifter1(vex::motor(vex::PORT11)), 
       lifter2(vex::motor(vex::PORT10)), 
       lift(vex::motor_group(lifter1, lifter2)), 
       rot(vex::rotation(vex::PORT19))
       { 
        globalPtr = this;
       };
       
       void init() override; 
       void periodic() override; 
       void updateTelemetry() override; 
       void stop() override; 

};


#endif