#include "path.h" 
#include "vex.h"

Path::Path(BezierCurve* curve, TrapezoidConstants motionConstants, PIDConstants pidconstants, double lookAheadDistance, double maxCentripAccel){ 
   this->lastTimestamp = Brain.Timer.time();
   this->curve = curve;  
   this->curveProfile = new TrapezoidalMotionProfile(motionConstants, curve->getPathLength(), lastTimestamp); 
   this->turnController = new pidcontroller(pidconstants, 0);  
   this->lDist = lookAheadDistance; 
   this->maxCentripetalAcceleration = maxCentripAccel;
}

PathFrameOutput Path::calculateFrameOutput(double x, double y, double heading, double timestamp){ 
   
   PathFrameOutput res;  
   BezierReferencePoint refPoint = findReferencePoint(x,y);
   
   double deltaTime = (timestamp - lastTimestamp) / 1000;
   double dist = sqrt(pow(x - refPoint.point[0],2) + pow(y - refPoint.point[1], 2)); //Distance between the setpoint and the current position
   
   dist = std::max<double>(dist, 1e-6); 
   
   double desiredHeading = (atan2(refPoint.point[1] - y, refPoint.point[0] - x) + M_PI) / (2 * M_PI) * 360; //At that point the robot would face the setpoint

   double angleDiff = desiredHeading - heading;   //How much the robot needed to turn in order to face the setpoint
   if (fabs(angleDiff) > 180){ 
      angleDiff =(-1 * copysign(1, angleDiff)) * (360 - fabs(angleDiff));
   }  

   double linearVelocity = lastVelocity + (getAcceleration(refPoint.progressT) * deltaTime);  //Only add a frame of acceleration  
   
   linearVelocity = std::min<double>(linearVelocity, deriveMaxVelocity(fabs(dist / std::max<double>(2 * sin(angleDiff / 360 * M_PI), 1e-6) )) );   
   
   double angularVelocity = turnController->calculate(angleDiff, Brain.Timer.time());  
    
   res.angularVelocity = angularVelocity; 
   res.linearVelocity = linearVelocity;  

   lastVelocity = linearVelocity; 
   pathProgress = refPoint.progressT; 

   lastTimestamp = timestamp;
   
   return res; 

} 

BezierReferencePoint Path::findReferencePoint(double x, double y){ //TO-DO
    BezierReferencePoint res;   
    
    double currentT = pathProgress; 
    
    double t_interval = (1.0/500);  

    currentT += t_interval; 

    double fallbackT = -1; 
    double smallestDist = -1;   

    double currentDist = 0; 

    bool withinRadialZone = false; 
    
    array<double,2> coordinates;  

    while (currentT < 1){ 
      curve->generatePoint(currentT, coordinates); 
      currentDist = sqrt(pow(x - coordinates[0], 2) + pow(y - coordinates[1], 2)); 

      if (smallestDist == -1 || currentDist < smallestDist){ 
         smallestDist = currentDist; 
         fallbackT = currentT;
      } 

      if (currentDist > this->lDist){ 
         if (withinRadialZone){ 
            break;
         }
      } else { 
         withinRadialZone = true;
      } 
      currentT += t_interval; 
    } 

    if (!withinRadialZone){ 
      currentT = fallbackT; 
    } 

    curve->generatePoint(currentT, coordinates); 

    res.point = coordinates; 
    res.progressT = currentT;
   
    return res;
} 

double Path::deriveMaxVelocity(double radius){ 
   return sqrt(maxCentripetalAcceleration * radius);
} 

void Path::setTimestamp(double stamp){ 
   this->lastTimestamp = stamp;
} 

double Path::getAcceleration(double progressT){ 
   // vo = lastVelocity 
   // xo = distance traveled on path
   // v = setpoint velocity
   // x = setpoint distance   

   double vo = lastVelocity; 
   double xo = curve->getPathLength() * progressT;   
   
   double x; //Pursuing dist
   double v; //At that dist what will the velocity be
   //Acceleration will always pursue reaching the phase velocity at the phase shift position 

   if (xo < curveProfile->getAccelDist()){ 
      x = curveProfile->getAccelDist();  
      v = curveProfile->getConstants().maxVelocity; //Pursue max velocity
   } else if (xo < curveProfile->getAccelDist() + curveProfile->getCruiseDist()){ 
      x = curveProfile->getAccelDist() + curveProfile->getCruiseDist(); 
      v = curveProfile->getConstants().maxVelocity; //Maintain max velocity
   } else { 
      x = curve->getPathLength(); 
      v = 0; //Slow down to rest
   } 
   double acceleration = (pow(v,2) - pow(vo, 2)) / (2*(x - xo));  
   return std::min<double>(fabs(acceleration), curveProfile->getConstants().maxAcceleration) * copysign(1, acceleration); //Cannot exceed maximum robot acceleration
   
}