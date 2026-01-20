#include "trapezoidalMotion.h" 
#include "math.h"

void TrapezoidalMotionProfile::init(){  
  
   accelTime = maxVelocity / maxAcceleration; 
   accelDist = (0.5 * maxAcceleration * pow(accelTime,2)); 

   if (accelDist > setpoint / 2){ 
     accelTime = sqrt((setpoint / maxAcceleration)); 
     accelDist = setpoint / 2; 
     maxVelocity = accelTime * maxAcceleration; 
   }  

   decelDist = accelDist; 
   decelTime = accelTime; 

   cruiseDist = setpoint - accelDist - decelDist; 
   cruiseTime = cruiseDist / maxVelocity;   

}; 

double TrapezoidalMotionProfile::calculateVelocity(double time){ 
    double elapsedTime = time - startingTimestamp;  
    double output;
    if (elapsedTime < accelTime)  
       output = maxAcceleration * elapsedTime;  // v = at
    else if (elapsedTime < accelTime + cruiseTime) 
       output = maxVelocity; 
    else if (elapsedTime < accelTime + cruiseTime + decelTime) { 
       output = maxVelocity - ((elapsedTime - accelTime - cruiseTime) * maxAcceleration); // v = v0 - at
    } else { 
       output = 0;
    }
    return output;
};    


double TrapezoidalMotionProfile::calculateAcceleration(double time){ 
  double elapsedTime = time - startingTimestamp; 
  double output;  
  if (elapsedTime < accelTime)  
       output = maxAcceleration;  
  else if (elapsedTime < accelTime + cruiseTime) 
       output = 0; 
  else if (elapsedTime < accelTime + cruiseTime + decelTime){ 
       output = -maxAcceleration;
  } else { 
       output = 0;
  }
  return output;
} 



double TrapezoidalMotionProfile::calculatePosition(double time){ 
  double output;  
  double elapsedTime = time - startingTimestamp; 
  if (elapsedTime < accelDist){
     output = (0.5 * maxAcceleration * pow(elapsedTime, 2)); // (1/2)at^2
  } else if (elapsedTime < accelTime + cruiseTime){
     output = (0.5 * maxAcceleration * pow(accelTime, 2)) + (maxVelocity * (elapsedTime - accelTime)); // vt
  } else if (elapsedTime < accelTime + cruiseTime + decelTime) { 
     double peakDist = ((0.5 * maxAcceleration * pow(accelTime, 2)) + (maxVelocity * cruiseTime)); //x0 + v0t + 1/2at^2
     double t = elapsedTime - cruiseTime - accelTime; 
     output = peakDist + ((maxVelocity * t) - (0.5 * maxAcceleration * pow(t, 2)));
  } else { 
     output = -1;
  };
  return output;
} 


TrapezoidalSetpoint TrapezoidalMotionProfile::generateSetpoint(double time){ 
   TrapezoidalSetpoint res;  
   res.position = calculatePosition(time); 
   res.velocity = calculateVelocity(time); 
   res.acceleration = calculateAcceleration(time); 
   return res;
}; 

void TrapezoidalMotionProfile::setPositionTolerance(double posTol){ 
   tolerancePos = posTol;
}; 

void TrapezoidalMotionProfile::setVelocityTolerance(double velTol){ 
   toleranceVel = velTol;
}; 

bool TrapezoidalMotionProfile::atGoal(double currentPosition, double currentVelocity){ 
   return fabs(currentPosition - setpoint) < tolerancePos && fabs(currentVelocity) < toleranceVel;
}; 

double TrapezoidalMotionProfile::getTotalDuration(){ 
   return accelTime + cruiseTime + decelTime;
}; 

double TrapezoidalMotionProfile::getStartTime(){ 
   return startingTimestamp; 
}