#include "trapezoidalMotion.h" 
#include "math.h"

void TrapezoidalMotionProfile::init(){  

   accelTime = maxVelocity / maxAcceleration; 
   accelDist = accelTime * maxVelocity; 

   if (accelDist > setpoint / 2){ 
     accelTime = sqrt((setpoint / 2) / (maxAcceleration / 2)); 
     accelDist = 0.5 * maxAcceleration * pow(accelTime, 2); 
     maxVelocity = accelTime * accelDist;
   } 

   decelDist = accelDist; 
   decelTime = accelTime; 

   cruiseDist = setpoint - accelDist - decelDist; 
   cruiseTime = cruiseDist / maxVelocity;  


}; 

double TrapezoidalMotionProfile::calculateVelocityT(double time){ 
    double elapsedTime = time - startingTimestamp;  
    double output;
    if (elapsedTime < accelTime)  
       output = (elapsedTime / accelTime) * maxVelocity;  
    else if (elapsedTime < accelTime + cruiseTime) 
       output = maxVelocity; 
    else { 
       output = -(1 - ((elapsedTime - cruiseTime - accelTime) / decelTime)) * maxVelocity; 
    } 
    return output;
};  

double TrapezoidalMotionProfile::calculateVelocityD(double dist){ 
  double output; 
  if (dist < accelDist) 
     output = (dist / accelDist) * maxVelocity; 
  else if (dist < accelDist + cruiseDist) 
     output = maxVelocity; 
  else { 
     output = -(1 - ((dist - cruiseDist - accelDist) / decelDist)) * maxVelocity;
  } 
  return output;
};  






