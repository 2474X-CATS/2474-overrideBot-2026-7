#include "lowpass.h" 

double LPFilter::calculate(double newMeasurement){  
  double res;
  if (std::isnan(lastMeasurement)){ 
    res = newMeasurement;
  } else { 
    res = conf * newMeasurement + ((1.0 - conf) * lastMeasurement); 
  } 
  lastMeasurement = newMeasurement; 
  return res;
} 

void LPFilter::setConfidence(double newConfidence){ 
   this->conf = newConfidence;
}