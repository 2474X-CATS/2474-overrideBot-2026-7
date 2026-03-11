#ifndef __PATH_H__ 
#define __PATH_H__ 

#include "../utilities/bezier.h" 
#include "trapezoidalMotion.h"  
#include "pidcontroller.h"
#include "math.h"

typedef struct { 
   double linearVelocity; //The calculated base speed derived from a trapzeoidal motion profile  
   double angularVelocity; //Based on the caluclated linear velocity and the angular error calculate the angular velocity 
} PathFrameOutput; 

typedef struct{ 
   array<double, 2> point; 
   double progressT; //Closest t point
} BezierReferencePoint;

class Path {   

   private:     

      BezierCurve* curve = nullptr;   
      pidcontroller* turnController = nullptr; 
      TrapezoidalMotionProfile* curveProfile = nullptr;   

      double lDist;  
      double pathProgress = 0;    
      double maxCentripetalAcceleration;  
      double lastVelocity = 0;  

      double lastTimestamp; 
     
      BezierReferencePoint findReferencePoint(double x, double y);  

      double deriveMaxVelocity(double radius);  

      double getAcceleration(double progressT); 

   public:  
      Path(BezierCurve* curve, TrapezoidConstants motionConstants, PIDConstants pidconstants, double lookAheadDistance, double maxCentripAccel);    

      PathFrameOutput calculateFrameOutput(double x, double y, double heading, double timestamp); 

      void setTimestamp(double stamp);

};




#endif 