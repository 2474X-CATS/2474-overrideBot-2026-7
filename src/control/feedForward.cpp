#include "feedForward.h"
#include "math.h"


double FFConstants::calculate(double velocity, double acceleration){ //m/s
    return kS * copysign(1, velocity) + kV * velocity + kA * acceleration;
} 

double AngularArmFFConstants::calculate(double theta, double velocity, double acceleration){ // rot/s   
    double static_output;  
    if (velocity == 0){ 
        static_output = 0; 
    } else { 
        static_output = (copysign(1, velocity)) * kS_rot;
    }
      
    return ((kCos * kCos_ratio) * cos(theta)) + static_output + (kV_rot * velocity) + (kA_rot * acceleration);
}

double ElevatorFFConstants::calculate(double velocity, double acceleration){ // m/s
    return kG + ffConsts.calculate(velocity, acceleration);
}