#include "matchloader.h" 

Matchloader *Matchloader::globalRef = nullptr; 

void Matchloader::init(){ 
   set<bool>("is_on", true);     
   stop();
}  

void Matchloader::periodic(){ 
   mlPiston.set(!RobotState::getStateOf("matchloader_out")); 
} 

void Matchloader::updateTelemetry(){ 
   return;
} 

void Matchloader::stop(){ 
   mlPiston.close();
}