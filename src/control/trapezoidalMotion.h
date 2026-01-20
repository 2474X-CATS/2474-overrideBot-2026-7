#ifndef __TRAP_MOTION_H__ 
#define __TRAP_MOTION_H__ 

typedef struct {  
   double position; 
   double velocity; 
   double acceleration; 
} TrapezoidalSetpoint;  

typedef struct { 
  double maxVelocity; 
  double maxAcceleration; 
} TrapezoidConstants;

class TrapezoidalMotionProfile {  
   
    private: 
      double accelDist; 
      double accelTime; 

      double cruiseDist; 
      double cruiseTime; 

      double decelDist; 
      double decelTime; 
      
      double maxVelocity; 
      double maxAcceleration; 
 
      double setpoint; 

      double startingTimestamp;  

      double toleranceVel = 0.0; 
      double tolerancePos = 0.0; 

      double calculateVelocity(double time); 
      double calculateAcceleration(double time);  

      void init(); 

    public: 

      TrapezoidalMotionProfile(TrapezoidConstants constants, double setpoint, double startingTime): 
      maxVelocity(constants.maxVelocity), 
      maxAcceleration(constants.maxAcceleration), 
      setpoint(setpoint), 
      startingTimestamp(startingTime)
      { 
        init();
      };   
      
      TrapezoidalSetpoint generateSetpoint(double time);

      void setPositionTolerance(double posTol); 
      void setVelocityTolerance(double velTol);   

      double getTotalDuration(); 
      double getStartTime();   

      double calculatePosition(double time); 

      bool atGoal(double currentPosition, double currentVelocity); 

};



#endif