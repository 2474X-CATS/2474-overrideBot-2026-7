#include "hooks.h" 

Hooks *Hooks::globalRef = nullptr; 

void Hooks::init(){ 
   set<bool>("is_on", true);     
   stop(); 
}  

void Hooks::periodic(){ 
   hookPiston.set(RobotState::getStateOf("descore_out")); 
} 

void Hooks::updateTelemetry(){ 
   return;
} 

void Hooks::stop(){ 
   hookPiston.set(false);
}