#include "claw.h" 

Claw* Claw::globalPtr = nullptr; 

double Claw::MAXIMUM_TOLERABLE_DISTANCE = 50; 

Claw& Claw::getObject(){ 
    return *globalPtr;
} 

void Claw::init(){ 
    return;
}  

void Claw::periodic(){ 
   clench(get<bool>("clenched")); 
   flip(get<bool>("facing_down")); 
} 

void Claw::updateTelemetry(){  
   stateControl();  
   set<bool>("senses_object", sensesObject()); 
   if (!RobotState::getStateOf("in_autonomous")){ 
     respondToRequests();
   }
   
}  

void Claw::stop(){ 
    clench(true);
} 

bool Claw::sensesObject(){ 
    return objectDetector.objectDistance(vex::distanceUnits::mm) <= MAXIMUM_TOLERABLE_DISTANCE; //If the claw is picking something up basically [Using sensors]
}

void Claw::stateControl(){ 
    set<bool>("clenched", get<bool>("senses_object"));
}

void Claw::respondToRequests(){   
     if (RobotState::getStateOf("awaiting_claw_act")){ 
          if (get<bool>("senses_object")){ 
             set<bool>("clenched", !get<bool>("clenched"));
          }
     }  

     if (RobotState::getStateOf("awaiting_flip")){ 
          set<bool>("facing_down", !get<bool>("facing_down"));  
     }  

     RobotState::manuallyModifyState("awaiting_claw_act", false); 
     RobotState::manuallyModifyState("awaiting_flip", false);
}
 
void Claw::clench(bool clenched){
    claw.set(!clenched);
} 

void Claw::flip(bool facingDown){ 
    if (RobotState::getStateOf("inverted")){ 
      facingDown = !facingDown;
    } 
    wrist.set(facingDown);
}