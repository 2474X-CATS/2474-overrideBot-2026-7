#include "trapezoidalMotion.h"
#include "math.h"

void TrapezoidalMotionProfile::init(double startingVelocity, double finalVelocity)
{
   this->startingVelocity = startingVelocity;

   phaseOneDirection = (int)(copysign(1, maxVelocity - startingVelocity));

   accelTime = fabs((maxVelocity - startingVelocity)) / maxAcceleration;
   accelDist = (startingVelocity * accelTime) + (0.5 * (phaseOneDirection * maxAcceleration) * pow(accelTime, 2));

   if (accelDist > setpoint / 2)
   {
      maxVelocity = sqrt(
          pow(startingVelocity, 2) + (2 * (phaseOneDirection * maxAcceleration) * (setpoint / 2)));
      consts.maxVelocity = maxVelocity;
      accelTime = fabs((maxVelocity - startingVelocity)) / maxAcceleration;
      accelDist = (startingVelocity * accelTime) + (0.5 * (phaseOneDirection * maxAcceleration) * pow(accelTime, 2));
   }

   phaseTwoDirection = (int)(copysign(1, finalVelocity - maxVelocity));

   decelTime = fabs((finalVelocity - maxVelocity)) / maxAcceleration;
   decelDist = (maxVelocity * decelTime) + (0.5 * (phaseTwoDirection * maxAcceleration) * pow(decelTime, 2));

   cruiseDist = setpoint - accelDist - decelDist;
   cruiseTime = cruiseDist / maxVelocity;
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