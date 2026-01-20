#ifndef __TRAJECTORY_CONTROL_H__ 
#define __TRAJECTORY_CONTROL_H__ 

#include "pidcontroller.h"
#include "feedForward.h"  
#include "trapezoidalMotion.h" 



class TrajectoryController {  
   
   private:  
     
     pidcontroller* controller;  
     FeedForward* feedForward; 
     TrapezoidalMotionProfile profile;  

     double setpointPos; 
     double setpointVel; 
     double setpointAcc;    

     double lastPos; 
     double lastVel;
    
     double calculateFFOutput(); 
     double calculatePDOutput(double currentPosition); 

   public:   
     
     TrajectoryController(PIDConstants pdConsts, FFConstants ffConsts, TrapezoidalMotionProfile profile) 
     : controller(new pidcontroller(pdConsts, 0)), 
       feedForward(new FeedForward(ffConsts)), 
       profile(profile)
      { 
       
      }; 

     void refreshSetpoints(double time);  

     void init(); 
     
     double calculate(double currentPosition);  

     bool atGoal();
     
     
      
      
   
   

};

#endif