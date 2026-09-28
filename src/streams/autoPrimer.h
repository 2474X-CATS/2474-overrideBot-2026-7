#ifndef __AUTO_PRIMER_H__ 
#define __AUTO_PRIMER_H__  

//#include "../architecture/dataStream.h"  
#include "odometry.h"
#include "../subsystems/elevator.h"
/*  

This class will dynamically set the setpoints of the elevator while cycling so cycle time is minimized. 
It works by settings setpoints using the sniper_score_enabled state in the elevator class in order to pursue  
stack heights before reaching points.  

Whenever a goal is scored on the registered height for that goal is incremented by the stack height. 

Goal targets are chosen in one of two ways by  
- Odometry (assuming drift is not significant) 
- Manual switching the target goal using state   

*/ 

// How the primer will choose setpoints   

typedef enum {
    LOCATION = 0, // Find the nearest goal 
    MANUAL_SHIFT   // Switch between points from the origin  
} DestinationProtocol;

// Represents a single goal that has a current height and location   

//[+5g, +6g, +7g, +8g, origin, +1g, +2g, +3g, +4g] 

/*
[
| A_GOAL; //All 
---------------> Start here
| N_GOAL;
| A_GOAL; //Opp
| N_GOAL;
| A_GOAL; //Opp
| N_GOAL;
---------------> Or here
| A_GOAL; //All
| N_GOAL; 
V
]
*/

extern double STACK_HEIGHT_MM;

typedef struct { 

   Location* goalPosition;  
   double goalHeight = Elevator::LEVELED_HEIGHT; 
   
   void setHeight(double height);
   double getHeight();
   double getDistance(double botX, double botY);
 
} Goal; 


class AutoPrimer : public DataStream {  

    public:
       
       AutoPrimer(): 
         DataStream( 
           "primer", 
           {
            (EntrySet){"priming_method", EntryType::INT}, //Able to be typecasted to a DestinationProtocol
            (EntrySet){"shift_direction", EntryType::INT},
            (EntrySet){"active", EntryType::BOOL}
           }
         )
       {};

       void refreshData() override; //Update based on the priming method  
       /* 
        If the robot is in possesion and priming  
             - Then update the front ran setpoint dynamically (if the last index is not equal to the current) 
        If the robot is in auto
          When the elevator is activated
             - Then switches a state for elevator pending 
          When the elevator is not activated if elevator pending  
             - Then record the height of the elevator when deactivated   
             - Pending is false  
       */
       void init() override; 
         
    private: 
       
       void locationBasedShiftUpdate(); //Reference the closest index
       void manualBasedShiftUpdate(); //Simply reference the correct index
       
       bool updateGoalIndex(); //Runs appropriate shift update function and modifies the elevator priming setpoint based on it
       void pasteSetpoints();  

       void replaceGoalHeight();

       int goalIndex = -1;
       int lastGoalIndex = -2; 

       Goal goals[8]; 
};



#endif 