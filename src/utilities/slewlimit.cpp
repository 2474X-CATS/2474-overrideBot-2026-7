#include "slewlimit.h" 
#include "../architecture/robotConfig.h" 

void SRLimiter::limit(double& output){  
    double delta = Brain.Timer.time() - lastTimestamp; //Millis 

    double maxVal = previousOutput + (maxJ * delta); 
    double minVal = previousOutput - (maxJ * delta);
    
    output = maxVal < output ? maxVal : (minVal > output ? minVal : output); 
    
    previousOutput = output;
    lastTimestamp = Brain.Timer.time();
} 

void SRLimiter::setPreviousOutput(double output){ 
     previousOutput = output;
} 

void SRLimiter::setTimestamp(double timestamp){ 
     lastTimestamp = timestamp;
}