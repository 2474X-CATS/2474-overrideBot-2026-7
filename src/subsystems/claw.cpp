#include "claw.h" 

Claw* Claw::globalPtr = nullptr;
double Claw::MAXIMUM_TOLERABLE_DISTANCE = 50; 
int Claw::SCORE_DELAY_MILLIS = 250;

Claw& Claw::getObject(){ 
    return *globalPtr;
}

void Claw::init(){ 
    return;
}

void Claw::periodic(){  
    clench(clenched);  
}  

bool Claw::sensesObject(){ 
    return objectDetector.objectDistance(vex::distanceUnits::mm) < MAXIMUM_TOLERABLE_DISTANCE;
}

void Claw::updateTelemetry(){
   set<bool>("in_possession", clenched && sensesObject()); 
   stateControl();
}  

void Claw::stop(){  
    claw.set(true);
    return;
} 


void Claw::stateControl(){  
   SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));   
   
   bool still = Telemetry::inst.getValueAt<bool>("ss_manager", "setpoints_reached");
   
   if (waiting){ 
      if (Brain.Timer.time() - lastScoreStamp >= SCORE_DELAY_MILLIS){ 
         set<bool>("active", false); 
         Telemetry::inst.placeValueAt<bool>(true, "forearm", "active");
         waiting = false;
      }
   } else if (pos == SuperStructurePosition::AUTO){ //Whenever macro is running
     if (get<bool>("active")){
         clenched = false; 
         waiting = true;
         lastScoreStamp = Brain.Timer.time();
     }
   } else {
      if (!still){  
        clenched = true; 
      } else {  
        switch (pos){ 
          case STANDING:
             clenched = RobotState::getStateOf("command_grip");
             break; 
          case GROUND:
             clenched = false; 
             break;
          case PRIMED:    
             clenched = true; 
             break;  
          default: 
             break;
        } 
      } 
   }   
}

void Claw::respondToRequests(){  

    SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));   
    bool still = Telemetry::inst.getValueAt<bool>("ss_manager","setpoints_reached");  

    if (still){  
        if (RobotState::getStateOf("awaiting_claw_act")){  
            if (pos != SuperStructurePosition::AUTO){ 
               set<bool>("requesting_act", true);
            }
        }  
        if (RobotState::getStateOf("awaiting_flip")){  
            if (pos == SuperStructurePosition::PRIMED){ 
               set<bool>("facing_down", !get<bool>("facing_down"));
            }
        }   

        RobotState::manuallyModifyState("awaiting_claw_act", false); 
        RobotState::manuallyModifyState("awaiting_flip", false); 

    } 
}
 
void Claw::clench(bool clenched){ 
    claw.set(!clenched);
} 
