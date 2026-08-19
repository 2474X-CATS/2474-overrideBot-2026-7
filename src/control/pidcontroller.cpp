#include <cstdlib>
#include "pidcontroller.h"

pidcontroller::pidcontroller(PIDConstants conts, double destination) : kP(conts.P),
                                                                       kI(conts.I),
                                                                       kD(conts.D),
                                                                       errorTolerance(conts.errorTolerance),
                                                                       iLimit(conts.iLimit), 
                                                                       minimumOutput(conts.minimumOutput)
{
   setpoint = destination;
};

pidcontroller::pidcontroller() : kP(0),
                                 kI(0),
                                 kD(0),
                                 errorTolerance(0),
                                 iLimit(0), 
                                 minimumOutput(0)
{
   setpoint = 0;
};

bool pidcontroller::atSetpoint(double position)
{
   return fabs(setpoint - position) <= errorTolerance;
};

double pidcontroller::calculate(double position, double timestamp)
{
   double dt = (timestamp - lastTimestamp) / 1000;
   lastTimestamp = timestamp;
   double error = setpoint - position;
   if (atSetpoint(position))
      return 0;
   integral += error * dt;
   if (iLimit > 0)
   {
      if (integral > iLimit)
         integral = iLimit;
      else if (integral < -iLimit)
         integral = -iLimit;
   } 
   if (prevError != 0){ 
      derivative = (error - prevError) / dt;
   } else { 
      derivative = 0;
   }
   prevError = error;
   double output = (kP * error) + (kI * integral) + (kD * derivative);  

   if (fabs(output) < minimumOutput){ 
      output = minimumOutput * copysign(1, output); 
   }  
   
   return output;
};

void pidcontroller::setLastTimestamp(double timestamp)
{
   lastTimestamp = timestamp;
};

double pidcontroller::getDerivative()
{
   return derivative;
}

double pidcontroller::getSetpoint()
{
   return setpoint;
}

void pidcontroller::reset()
{
   integral = 0;
   derivative = 0;
   prevError = 0;
} 

double errorcontroller::calculate(double currentRef,  double timestamp){  
  if (!active){ 
   return 0;
  }
  return pidcontroller::calculate(referencePos - currentRef, timestamp);
}  

void errorcontroller::setReference(double ref){ 
   this->referencePos = ref; 
   active = true;
} 

void errorcontroller::deactivate(){ 
   active = false; 
   reset(); 
}

