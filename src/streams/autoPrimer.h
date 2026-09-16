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
    TOGGLE,   // Switch between points from the origin
    SILENT    // Setpoints are not preset
} DestinationProtocol;

// Represents a single goal that has a current height and location   

//[+5g, +6g, +7g, +8g, origin, +1g, +2g, +3g, +4g]


double STACK_HEIGHT_MM;

typedef struct { 

   Location* goalPosition; 
   double goalHeight = Elevator::LEVELED_HEIGHT;
   
   void score();
   double getHeight();
   double getDistance(double botX, double botY);
 
} Goal; 


class AutoPrimer : public DataStream {  

    public:
       
       AutoPrimer(): 
         DataStream( 
           "primer", 
           {  
             (EntrySet){"front_ran_setpoint", EntryType::DOUBLE}, //Sent to the elevator
             (EntrySet){"goal_index", EntryType::INT}
           }
         )
       {};

       void refreshData() override; // Calls every telemetry frame
       void init() override; // Sets up sensors for data-collection
    
    private:   
       int shiftVal = 1;
       Goal[] goals; 
      

};



#endif 