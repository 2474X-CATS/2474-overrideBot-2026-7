#ifndef __ELEVATOR_H__ 
#define __ELEVATOR_H__ 

#include "../architecture/subsystem.h"   

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

       ElevatorState currentState = ElevatorState::E_HOLDING; 

       int raisingDirection = 0;
       int setpointDirection;

       double previousTimestamp;

       TrapezoidalMotionProfile* motionProfile = nullptr; //When the robot has defined setpoints it needs to reach
       TrapezoidConstants motionConsts; 
       
       bool reachedSetpoint();  

       void stateControl();
       void respondToRequests();  
       
       void setSetpoint(double setpoint);
       void updatePosition(double velocity); 


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
            (EntrySet){"sniper_score_enabled", EntryType::BOOL}, 
            (EntrySet){"lifting_timestamp", EntryType::DOUBLE}, 
            (EntrySet){"hold", EntryType::BOOL}, 
            (EntrySet){"task_id", EntryType::INT}
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