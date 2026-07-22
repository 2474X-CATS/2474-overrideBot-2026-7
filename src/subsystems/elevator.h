#ifndef __ELEVATOR_H__ 
#define __ELEVATOR_H__ 

#include "../architecture/subsystem.h"   

#include "../control/feedForward.h" 
#include "../control/pidcontroller.h" 
#include "../control/trapezoidalMotion.h"


typedef enum { 
   E_HOLDING, 
   E_PRIMING, 
   E_PURSUING
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

       int raisingDirection = 0;
       int setpointDirection;  

       double previousHeight; 
       double previousTimestamp;  

       ElevatorState currentState = ElevatorState::E_HOLDING;

       TrapezoidalMotionProfile* motionProfile = nullptr; //When the robot has defined setpoints it needs to reach
       TrapezoidConstants motionConsts; 
       
       errorcontroller* correctionController = nullptr; //Adjusting the output of ff so its more accurate
      
       ElevatorFFConstants elevatorFF; //Elevator FF Running at almost all times
       
       vex::motor lifter1; 
       vex::motor lifter2;  

       vex::motor_group lift;  

       vex::distance primingSensor;

       vex::rotation rot; 

       double calculateOutput(double velocity, double acceleration); 
       bool reachedSetpoint(); 

       void stateControl();
       void respondToRequests();  
       
       double getPosition();  
       double getVelocity();

       void setSetpoint(double setpoint);  
       void updatePosition();

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
            (EntrySet){"priming_setpoint", EntryType::DOUBLE},
            (EntrySet){"requested_height", EntryType::DOUBLE}, //Specifically what height do we want to reach 
            (EntrySet){"sensing_stack", EntryType::BOOL},
            (EntrySet){"current_height", EntryType::DOUBLE},
            (EntrySet){"priming_direction", EntryType::INT}, 
            (EntrySet){"sniper_score_enabled", EntryType::BOOL}
         }
       ),
       lifter1(vex::motor(vex::PORT11)), 
       lifter2(vex::motor(vex::PORT10)), 
       lift(vex::motor_group(lifter1, lifter2)), 
       rot(vex::rotation(vex::PORT19)), 
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