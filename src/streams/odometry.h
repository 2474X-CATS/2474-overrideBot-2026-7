#ifndef __ODOM_H__ 
#define __ODOM_H__ 

#include "../architecture/dataStream.h" 

class Odometry : public DataStream {  

    public: 

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

       vex::inertial gyro; 
       vex::rotation rot;  

       double lastTimestamp = 0; 
       double lastPosition;

};



#endif