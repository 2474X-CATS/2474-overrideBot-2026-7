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

typedef enum { 
  RIGHT,
  LEFT,
  FRONT, 
  BACK,
} ResetFace;

class Odometry : public DataStream {  

    public: 
       
       static Location* getLocation(int index);  

       Odometry();
       Odometry(bool usingTendy);

       void refreshData() override; // Calls every telemetry frame
       void init() override; // Sets up sensors for data-collection 

    private:

       double xCenterOfRot;
       double yCenterOfRot;

       void calibratePerspective();

       bool initialized = false;

       static const double INERTIAL_WHEEL_RADIUS;   
       static const double ANG_ROT_DIST_FROM_CENTER;
       static const double GOAL_WIDTH; 
       static const double MATCHLOAD_CLEARANCE_MM;  

       static const double OFFSET_COR_X; 
       static const double OFFSET_COR_Y; 

       static Location* locations[];    

       //static Odometry* obj;
       
       double getApplicableWallHeading(double currentX, double currentY, double currentHeading, double xOffset, double yOffset); 
      
       //------------------------------------------------------- 
      
       double getStrafeVelocity(); //Side to side Displacement (make up for drift)

       double getForwardVelocity(); //Forward Displacement

       double getOmega(); //The angular displacement from last frame 

       double getProcessedHeading(); 
       
       //-------------------------------------------------------- 

       void generateReckoningTranslations(double& xTrans, double& yTrans, double delta); 
      
       //-------------------------------------------------------- 
        
       void resetOffGoal(int goal, double& xPos, double& yPos);

       //-------------------------------------------------------- 
       void deriveSimpleTranslations(double& xTrans, double &yTrans, double forwardDist, double strafeDist, double heading); 
        
       void deriveArcTranslations(double& xTrans, double& yTrans, double forwardDist, double strafeDist, double thetaTrans, double heading); 
        
       //-------------------------------------------------------
       void findTranslations(double& xCoords, double& yCoords, int& xCount, int& yCount, vex::distance& sensor, double angleOffset, double xOffset, double yOffset);  
       
       void laserCalibrate(double& resX, double& resY); 

       void snapHeading();

       double getHeading();
       
       vex::inertial gyro; 
       vex::rotation linRot;   
       vex::rotation angRot; 
      
       vex::distance frontDist; 
       vex::distance leftDist; 
       vex::distance rightDist;  
       vex::distance backDist; 

       double lastTimestamp = 0; 
       
}; 



#endif