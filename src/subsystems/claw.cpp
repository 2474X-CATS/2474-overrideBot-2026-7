#include "claw.h" 

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
   if (!RobotState::getStatef("in_autonomous")){ 
     respondToRequests();
   }
   
}  

void Claw::stop(){ 
    return;
} 

bool Claw::canGrab(){ 
    return false; //If the claw is picking something up basically [Using sensors]
}

void Claw::stateControl(){ 
   SuperStructurePosition pos = Telemetry::inst.getValueAt<int>("ss_manager", "position");   
   bool still = Telemetry::inst.getValueAt<bool>("ss_manager","setpoints_reached"); 

   if (pos == SuperStructurePosition::AUTO){ //Whenever macro is running
     if (get<bool>("active")){ 
         set<bool>("clenched", false); 
         set<bool>("active", false); 
         Telemetry::inst.placeValueAt<bool>(true, "forearm", "active"); 
     }
   } else {  

     if (!still){ 

       if (pos == SuperStructurePosition::GROUND || pos == SuperStructurePosition::STANDING){ 
         set<bool>("clenched", false);
       } else { 
         set<bool>("clenched", true);
       }  

       if (pos == SuperStructurePosition::STANDING){ 
         set<bool>("facing_down", true);
       } else { 
         set<bool>("facing_down", false);
       }  

     } else {   

       if (canGrab()){ 
         set<bool>("clenched", true);
       }   

     } 
   }
}

void Claw::respondToRequests(){  

    SuperStructurePosition pos = Telemetry::inst.getValueAt<int>("ss_manager", "position");   
    bool still = Telemetry::inst.getValueAt<bool>("ss_manager","setpoints_reached");  

    if (pos == SuperStructurePosition::PRIMED && still){  

        if (RobotState::getStateOf("awaiting_drop")){ 
            set<bool>("clenched", false);  
        }  

        if (RobotState::getStateOf("awaiting_flip")){ 
            set<bool>("facing_down", !get<bool>("facing_down"));  
             
        }  

        RobotState::manuallyModifyState("awaiting_drop", false); 
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