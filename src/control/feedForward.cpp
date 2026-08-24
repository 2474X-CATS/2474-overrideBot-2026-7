#include "feedForward.h"
#include "math.h"


double FFConstants::calculate(double velocity, double acceleration){ //m/s
    return kS * copysign(1, velocity) + kV * velocity + kA * acceleration;
} 

double AngularArmFFConstants::calculate(double absPosition, double velocity, double acceleration){ // rot/s
    return ((kCos * kCos_ratio) * cos(absPosition * 2 * M_PI)) + (kS_rot * copysign(1, velocity)) + (kV_rot * velocity) + (kA_rot * acceleration);
}

double ElevatorFFConstants::calculate(double velocity, double acceleration){ // m/s
    return kG + ffConsts.calculate(velocity, acceleration);
}
