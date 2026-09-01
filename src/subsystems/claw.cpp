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
   /*
   if (!RobotState::getStateOf("in_autonomous")){ 
     respondToRequests();
   } 
  */
}  

void Claw::stop(){ 
    return;
} 


void Claw::stateControl(){  

   SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));   
   
   bool still = Telemetry::inst.getValueAt<bool>("ss_manager", "setpoints_reached");
   
   if (pos == SuperStructurePosition::AUTO){ //Whenever macro is running
     if (get<bool>("active")){
         set<bool>("clenched", false); 
         set<bool>("active", false); 
         Telemetry::inst.placeValueAt<bool>(true, "forearm", "active"); 
     }
   } else {   
    if (!still){  
       set<bool>("clenched", true); 
       //set<bool>("requesting_act", false);    
       if (!get<bool>("senses_object") && get<bool>("facing_down")){ 
         set<bool>("facing_down", false); 
       }
    } else {  
        switch (pos){ 
          case STANDING:    
             set<bool>("facing_down", true);
             if (get<bool>("senses_object")){ 
               set<bool>("clenched", true);
             } else { 
               set<bool>("clenched", false);
             } 
             break; 
          case GROUND:   
             set<bool>("clenched", false); 
             break;
          case PRIMED:    
             set<bool>("clenched", true); 
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
    return;
} 

void Claw::flip(bool facingDown){   
    return;
}