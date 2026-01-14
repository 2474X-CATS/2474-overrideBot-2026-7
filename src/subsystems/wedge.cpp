#include "wedge.h"  

Wedge *Wedge::globalRef = nullptr; 

void Wedge::init(){ 
   set<bool>("is_on", true);     
   stop(); 
}  

void Wedge::periodic(){     
   
   wedgePiston2.set(RobotState::getStateOf("elevated")); 
   wedgePiston1.set(RobotState::getStateOf("elevated")); 

   if (wedgeSensor.objectDistance(vex::distanceUnits::cm) < 2.5 && (!RobotState::getStateOf("release_grip"))){ 
     wedgeCloser.set(false);  
   } else {   
     wedgeCloser.set(true);
   } 
} 

void Wedge::updateTelemetry(){ 
   return;
} 

void Wedge::stop(){ 
   wedgePiston1.set(false);  
   wedgePiston2.set(false); 
}