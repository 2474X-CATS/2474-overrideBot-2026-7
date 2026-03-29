#ifndef __PID_H__
#define __PID_H__ 

#include "math.h" 

/* 
Uses Proportional Integral Derviative control to insure motion smoothness,
and setpoint precision (given some tuning)
*/
typedef struct
{
  double P;
  double I;
  double D;
  double iLimit = -1;
  double errorTolerance = 0;
} PIDConstants;

class pidcontroller
{ 

private: 

  double kP;
  double kI;
  double kD;

  double lastTimestamp = 0;

  double integral = 0;
  double derivative = 0;

  double errorTolerance;
  double iLimit;

  double prevError = 0;

  double setpoint;

public: 

  pidcontroller(PIDConstants conts, double destination);

  pidcontroller();

  double calculate(double position, double timestamp);

  bool atSetpoint(double position);

  double getSetpoint();

  double getDerivative();

  void setSetpoint(double setpoint);

  void setLastTimestamp(double timestamp);

  void reset();
};  

class errorcontroller : public pidcontroller {  

  private: 
     double referencePos = 0; 
  
  public:   
     errorcontroller(PIDConstants consts) : pidcontroller(consts, 0){};  
    
     double calculate(double currentRef, double timestamp); 
     
     void setReference(double ref);
}  



#endif