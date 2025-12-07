#ifndef __TRAP_MOTION_H__ 
#define __TRAP_MOTION_H__ 


typedef struct {  

    double maxVelocity; 
    double maxAcceleration; 
    
} TrapezoidConstants; 

class TrapezoidalMotionProfile {  
    private:  
      double setpoint;    

      double startingTimestamp;  

      double maxVelocity; 
      double maxAcceleration;

      double accelTime; 
      double decelTime;  
      double cruiseTime;

      double accelDist; 
      double decelDist; 
      double cruiseDist;

      void init();  

    public:  

      TrapezoidalMotionProfile(TrapezoidConstants constants, double setpoint, double startingTime): 
      startingTimestamp(startingTime), 
      setpoint(setpoint),
      maxVelocity(constants.maxVelocity), 
      maxAcceleration(constants.maxAcceleration)
      { 
        init();
      };   
      
      double calculateVelocityT(double time);  
  
      double calculateVelocityD(double dist);  
      

};



#endif