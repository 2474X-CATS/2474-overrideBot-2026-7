#include "claw.h" 

Claw* Claw::globalPtr = nullptr;
double Claw::MAXIMUM_TOLERABLE_DISTANCE = 0.0;

Claw& Claw::getObject(){ 
    return *globalPtr;
} 

void Claw::init(){ 
     set<bool>("senses_object", false);
}

void Claw::periodic(){  
    return;
} 

void Claw::updateTelemetry(){   
   stateControl(); 
   if (!RobotState::getStateOf("in_autonomous")){ 
     respondToRequests();
   }
}  

void Claw::stop(){ 
    return;
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
          if (get<bool>("senses_object")){
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
    return;
} 

void Claw::flip(bool facingDown){   
    return;
}