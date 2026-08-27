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
   set<bool>("senses_object", sensesObject());
   stateControl(); 
   if (!RobotState::getStateOf("in_autonomous")){ 
     respondToRequests();
   }
}  

void Claw::stop(){ 
    clench(false);
} 

bool Claw::sensesObject(){ 
    return objectDetector.objectDistance(vex::distanceUnits::mm) <= MAXIMUM_TOLERABLE_DISTANCE; //If the claw is picking something up basically [Using sensors]
}

void Claw::stateControl(){ 
   SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));   
   
   bool still = Telemetry::inst.getValueAt<bool>("forearm","at_setpoint") && Telemetry::inst.getValueAt<bool>("elevator", "at_setpoint"); 
   
   if (pos == SuperStructurePosition::AUTO){ //Whenever macro is running
     if (get<bool>("active")){
         set<bool>("clenched", false); 
         set<bool>("active", false); 
         Telemetry::inst.placeValueAt<bool>(true, "forearm", "active"); 
     }
   } else {   
    if (!still){  
       set<bool>("clenched", true);
       set<bool>("requesting_act", false); 

       if (pos == SuperStructurePosition::STANDING){ 
         set<bool>("facing_down", true);
       } else { 
         set<bool>("facing_down", false);
       }  
       
    } else {    
        if (pos == SuperStructurePosition::GROUND || pos == SuperStructurePosition::STANDING){ 
          set<bool>("clenched", false);
        } 

        if (get<bool>("requesting_act")){ 
          set<bool>("requesting_act", false);   
          if (sensesObject()){
            set<bool>("clenched", !get<bool>("clenched")); 
          }
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

void Claw::flip(bool facingDown){ 
    if (RobotState::getStateOf("inverted")){ 
      facingDown = !facingDown;
    } 
    wrist.set(facingDown);
}