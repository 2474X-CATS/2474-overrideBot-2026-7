#ifndef __FEED_FORWARD_H__
#define __FEED_FORWARD_H__

typedef struct
{
   double kS; // overcome static friction
   double kV; // maintain velocity
   double kA; // increase velocity  

   double calculate(double velocity, double acceleration);  

} FFConstants;  


typedef struct {  
   double kS_rot;
   double kV_rot; 
   double kA_rot;  
   double kCos;  
   double kCos_ratio = 1;  

   double calculate(double theta, double velocity, double acceleration);

} AngularArmFFConstants; 


typedef struct { 
   FFConstants ffConsts;
   double kG; 

   double calculate(double velocity, double acceleration);  

} ElevatorFFConstants;


/*
class FeedForward
{
   double kS;
   double kV;
   double kA;

public:
   FeedForward(FFConstants consts) : kS(consts.kS),
                                     kV(consts.kV),
                                     kA(consts.kA) {};     

   FeedForward(double cS, double cV, double cA) : kS(cS),
                                                  kV(cV),
                                                  kA(cA) {};   

   double calculate(double velocity);
   double calculate(double velocity, double accleration);
};  
*/ 


#endif