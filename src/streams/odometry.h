#ifndef __ODOM_H__ 
#define __ODOM_H__ 

#include "../architecture/dataStream.h"  
#include "../utilities/location.h" 

typedef enum { 
  ALL_NAT_NEU = 0, 
  ALL_NAT_ALL, 
  ALL_FOR_ALL, 
  ALL_FOR_NEU, 
  OPP_NAT_NEU, 
  OPP_NAT_ALL, 
  OPP_FOR_ALL, 
  OPP_FOR_NEU, 
  CENTRAL_GOAL, 
  ML_BL, 
  ML_BR, 
  ML_TL, 
  ML_TR
} Setpoint;

class Odometry : public DataStream {  

    public: 
       
       Location* getLocation(int index);  

       Odometry() : 
       DataStream( 
         "odometry", 
         {  
           (EntrySet){"starting_left", EntryType::BOOL}, 

           (EntrySet){"x_position_mm", EntryType::DOUBLE}, 
           (EntrySet){"y_position_mm", EntryType::DOUBLE}, 
           (EntrySet){"heading_deg", EntryType::DOUBLE},  

           (EntrySet){"velocity_ms", EntryType::DOUBLE},  
           
           (EntrySet){"immediate_distance", EntryType::DOUBLE},  

           (EntrySet){"oriented_c", EntryType::BOOL}
         }
       ), 
       gyro(vex::inertial(vex::PORT1)), 
       rot(vex::rotation(vex::PORT2)) 
       {};

       void refreshData() override; // Calls every telemetry frame
       void init() override; // Sets up sensors for data-collection

    private: 
       
       void setStartingOdometry();

       static double INERTIAL_WHEEL_RADIUS;  
       static double GOAL_WIDTH;

       static Location* locations[];   

       static void calibratePerspective();

       vex::inertial gyro; 
       vex::rotation rot;  

       double lastTimestamp = 0; 
       

};



#endif