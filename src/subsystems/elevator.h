#ifndef __ELEVATOR_H__ 
#define __ELEVATOR_H__ 

#include "../architecture/subsystem.h"   

#include "../control/feedForward.h" 
#include "../control/pidcontroller.h" 
#include "../control/trapezoidalMotion.h"

class Elevator : public Subsystem { 
    
    private:   
       
       static Elevator* globalPtr = nullptr;  
        
       static double GROUND_HEIGHT; 
       static double ELEVATOR_ERROR_TOLERANCE; 
       static double STACK_HEIGHT;

       TrapezoidalMotionProfile* motionProfile = nullptr; //When the robot has defined setpoints it needs to reach
       errorcontroller* correctionController = nullptr; //Adjusting the output of ff so its more accurate
       ElevatorFFConstants* elevatorFF = nullptr; //Elevator FF Running at almost all times

       double calculateOutput(); 
       
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
            (EntrySet){"current_height", EntryType::DOUBLE}, //Recorded height of the elevator
            (EntrySet){"at_setpoint", EntryType::BOOL}, //Achieved setpoint or no setpoint?
            (EntrySet){"height_setpoint", EntryType::DOUBLE}, //Height we want to be at
            (EntrySet){"has_setpoint", EntryType::BOOL}, //Do we have a setpoint we want to pursue?  
            (EntrySet){"current_velocity", EntryType::DOUBLE}, //Speed the elevator is going up or down (positive = up)
            (EntrySet){"on_ground", EntryType::BOOL} //Are we on the ground
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