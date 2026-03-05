#ifndef __PATH_H__ 
#define __PATH_H__ 

#include "bezier.h" 
#include "trapezoidalMotion.h" 
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
      TrapezoidalMotionProfile* curveProfile = nullptr; 
     
      double lDist;  
      double pathProgress = 0;   
      //double lastAngleSetpoint = -1; //Obscure number not likely to be replicated
      
      BezierReferencePoint findReferencePoint(double x, double y); 

   public:  
      Path(BezierCurve* curve, TrapezoidConstants constants, double lookAheadDistance){};    

      PathFrameOutput calculateFrameOutput(double x, double y, double heading);

};




#endif 