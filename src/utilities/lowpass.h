#ifndef __LOW_PSS__ 
#define __LOW_PSS__  

#include <cmath>


class LPFilter { 
    private:  
       
       double lastMeasurement = nan("");
       double conf; // <- Confidence level

    public:  
       LPFilter(double sensorConfidence): 
       conf(sensorConfidence){}; 

       double calculate(double newMeasurement); 
       void setConfidence(double newConfidence); 
};


#endif