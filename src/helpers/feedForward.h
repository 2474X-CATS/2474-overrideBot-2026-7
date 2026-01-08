#ifndef __FEED_FORWARD_H__ 
#define __FEED_FORWARD_H__ 


typedef struct { 
   double kS; //over come static friction
   double kV; //maintain kinetic friction
   double kA; // increase velocity
} FFConstants; 


class FeedForward {   

    double kS; 
    double kV; 
    double kA; 

    public:   
      FeedForward(FFConstants consts) : 
      kS(consts.kS), 
      kV(consts.kV), 
      kA(consts.kA)
      {};  

      double calculate(double velocity);  
      double calculate(double velocity, double accleration); 
      
};

#endif