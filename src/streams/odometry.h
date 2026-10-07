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
       
       static Location* getLocation(int index);  

       Odometry();

       void refreshData() override; // Calls every telemetry frame
       void init() override; // Sets up sensors for data-collection

    private: 
       
       void setStartingOdometry();

       static const double INERTIAL_WHEEL_RADIUS;   
       static const double ANG_ROT_DIST_FROM_CENTER;
       static const double GOAL_WIDTH; 
       static const double MATCHLOAD_CLEARANCE_MM; 

       static Location* locations[];   

       void calibratePerspective();
       
       void findTranslations(double& xTranslate, double& yTranslate, vex::distance sensor, double angleOffset, double distanceOffset);  
       
       double getHeading();
       
       vex::inertial gyro; 
       vex::rotation linRot;   
       vex::rotation angRot; 

       vex::distance frontDist; 
       vex::distance leftDist; 
       vex::distance rightDist; 

       double lastTimestamp = 0; 
       

};



#endif