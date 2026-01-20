#include "feedForward.h" 
#include "math.h" 

double FeedForward::calculate(double velocity, double acceleration){ 
   return kS * copysign(1, velocity) + kV * velocity + kA * acceleration;
};  

double FeedForward::calculate(double velocity){ 
    return calculate(velocity, 0);
}; 

