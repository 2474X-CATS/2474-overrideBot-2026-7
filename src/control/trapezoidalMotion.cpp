#include "trapezoidalMotion.h"
#include "math.h" 
#include "../architecture/robotConfig.h"

void TrapezoidalMotionProfile::init(double startingVelocity, double finalVelocity)
{  
   this->startingVelocity = startingVelocity;
    
   maxVelocity = setpoint < 0 ? maxVelocity * -1 : maxVelocity; 

   phaseOneDirection = (int)(copysign(1, maxVelocity - startingVelocity)); 

   //Brain.Screen.printAt(20, 60, "Direction p1: %d", phaseOneDirection); 
   
   accelTime = fabs((maxVelocity - startingVelocity)) / maxAcceleration; 

   //Brain.Screen.printAt(20, 80, "Time for initial accel: %.2f", accelTime);  

   accelDist = (startingVelocity * accelTime) + (0.5 * (phaseOneDirection * maxAcceleration) * pow(accelTime, 2));
   //Brain.Screen.printAt(20, 100, "Setpoint / AccelDist: %.2f", setpoint / accelDist);  

   if (setpoint / accelDist < 2)
   {  
      //Brain.Screen.printAt(20, 120, "Didn't accel on time"); 
      maxVelocity = sqrt(
          pow(startingVelocity, 2) + (2 * (phaseOneDirection * maxAcceleration) * (setpoint / 2))) * phaseOneDirection; 
      //Brain.Screen.printAt(20, 140, " New Max Velocity: %.2f", maxVelocity); 
      
      accelTime = fabs((maxVelocity - startingVelocity)) / maxAcceleration; 
      //Brain.Screen.printAt(20, 160, " New Accel time: %.2f", accelTime);
      accelDist = (startingVelocity * accelTime) + (0.5 * (phaseOneDirection * maxAcceleration) * pow(accelTime, 2)); 
      //Brain.Screen.printAt(20, 180, " New Accel Dist: %.2f", accelDist);
   } 

   phaseTwoDirection = (int)(copysign(1, finalVelocity - maxVelocity));
   //Brain.Screen.printAt(20, 200, "Direction p2: %d", phaseTwoDirection);  

   decelTime = fabs((finalVelocity - maxVelocity)) / maxAcceleration; 
   //Brain.Screen.printAt(20, 220, "Decel time: %.2f", decelTime); 

   decelDist = (maxVelocity * decelTime) + (0.5 * (phaseTwoDirection * maxAcceleration) * pow(decelTime, 2));
   //Brain.Screen.printAt(20, 220, "Decel Dist: %.2f", decelDist); 

   cruiseDist = setpoint - accelDist - decelDist;
   cruiseTime = cruiseDist / maxVelocity; 
   Brain.Screen.printAt(20, 220, "Cruise Dist: %.2f", cruiseTime);
};

double TrapezoidalMotionProfile::calculateVelocity(double time)
{
   double elapsedTime = (time - startingTimestamp) / 1000;
   double output;
   if (elapsedTime >= 0 && elapsedTime < accelTime)
      output = startingVelocity + (maxAcceleration * phaseOneDirection * elapsedTime); // v = at
   else if (elapsedTime < accelTime + cruiseTime)
      output = maxVelocity;
   else if (elapsedTime < accelTime + cruiseTime + decelTime)
   {
      output = maxVelocity + ((elapsedTime - accelTime - cruiseTime) * phaseTwoDirection * maxAcceleration); // v = v0 + at
   }
   else
   {
      output = 0;
   }
   return output;
};

double TrapezoidalMotionProfile::calculateAcceleration(double time)
{
   double elapsedTime = (time - startingTimestamp) / 1000;
   double output;
   if (elapsedTime >= 0 && elapsedTime < accelTime)
      output = phaseOneDirection * maxAcceleration;
   else if (elapsedTime < accelTime + cruiseTime)
      output = 0;
   else if (elapsedTime < accelTime + cruiseTime + decelTime)
   {
      output = phaseTwoDirection * maxAcceleration;
   }
   else
   {
      output = 0;
   }
   return output;
}

double TrapezoidalMotionProfile::calculatePosition(double time)
{
   double output;
   double elapsedTime = (time - startingTimestamp) / 1000;
   if (elapsedTime >= 0 && elapsedTime < accelTime)
   {
      output = (startingVelocity * time) + (0.5 * (phaseOneDirection * maxAcceleration) * pow(elapsedTime, 2)); // v0t + (1/2)at^2
   }
   else if (elapsedTime < accelTime + cruiseTime)
   {
      output = accelDist + (maxVelocity * (elapsedTime - accelTime)); // vt
   }
   else if (elapsedTime < accelTime + cruiseTime + decelTime)
   {
      double peakDist = accelDist + cruiseDist; // x0 + v0t + 1/2at^2
      double t = elapsedTime - cruiseTime - accelTime;
      output = peakDist + ((maxVelocity * t) + (0.5 * (phaseTwoDirection * maxAcceleration) * pow(t, 2)));
   }
   else
   {
      output = -1;
   };
   return output;
}

TrapezoidalSetpoint TrapezoidalMotionProfile::generateSetpoint(double time)
{
   TrapezoidalSetpoint res;
   res.position = calculatePosition(time);
   res.velocity = calculateVelocity(time);
   res.acceleration = calculateAcceleration(time);
   return res;
};

void TrapezoidalMotionProfile::setPositionTolerance(double posTol)
{
   tolerancePos = posTol;
};

void TrapezoidalMotionProfile::setVelocityTolerance(double velTol)
{
   toleranceVel = velTol;
};

bool TrapezoidalMotionProfile::atGoal(double currentPosition, double currentVelocity)
{
   return fabs(currentPosition - setpoint) < tolerancePos && fabs(currentVelocity) < toleranceVel;
};

double TrapezoidalMotionProfile::getTotalDuration()
{
   return (accelTime + cruiseTime + decelTime) * 1000; // In millis
};

double TrapezoidalMotionProfile::getStartTime()
{
   return startingTimestamp;
}

double TrapezoidalMotionProfile::getAccelDist()
{
   return accelDist;
}

double TrapezoidalMotionProfile::getDecelDist()
{
   return decelDist;
}

double TrapezoidalMotionProfile::getCruiseDist()
{
   return cruiseDist;
}

double TrapezoidalMotionProfile::getMaxVelocity()
{
   return maxVelocity;
}

double TrapezoidalMotionProfile::getMaxAcceleration()
{
   return maxAcceleration;
}

void TrapezoidalMotionProfile::setLastTimestamp(double timestamp)
{
   this->startingTimestamp = timestamp;
}