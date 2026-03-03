#include "catapult.h" 

Catapult *Catapult::globalRef = nullptr; 

void Catapult::init(){ 
   set<bool>("is_on", true);     
   stop(); 
}  

void Catapult::periodic(){ 
   return;
} 

void Catapult::updateTelemetry(){ 
   return;
} 

void Catapult::stop(){ 
   return;
}