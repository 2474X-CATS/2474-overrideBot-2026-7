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
   
   //clench(get<bool>("clenched")); 
   //flip(get<bool>("facing_down"));   
   
} 

void Claw::updateTelemetry(){   
   stateControl(); 
   if (!RobotState::getStateOf("in_autonomous")){ 
     respondToRequests();
   }
}  

void Claw::stop(){ 
    //clench(false); 
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
       
       if (pos == SuperStructurePosition::GROUND && get<bool>("senses_object")){ 
          set<bool>("clenched", false);
       }

       if (pos == SuperStructurePosition::STANDING){ 
         set<bool>("facing_down", true);
       } else { 
         set<bool>("facing_down", false);
       }  
       
    } else {    
        bool willClench; 
        if (pos == SuperStructurePosition::PRIMED){ 
           willClench = true;
        } else { 
           willClench = false;
        }  
        //willClench = !willClench [essentially approves the action]
        if (get<bool>("requesting_act")){ 
          set<bool>("requesting_act", false);   
          if (get<bool>("senses_object")){   
            willClench = !willClench;
          }
        }   

        set<bool>("clenched", willClench);
    } 
      
   }   

}

void Claw::respondToRequests(){  

    SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));   
    bool still = Telemetry::inst.getValueAt<bool>("forearm","at_setpoint") && Telemetry::inst.getValueAt<bool>("elevator", "at_setpoint"); 

    if (pos == SuperStructurePosition::PRIMED && still){  

        if (RobotState::getStateOf("awaiting_claw_act")){ 
            set<bool>("requesting_act", true);  
        }  

        if (RobotState::getStateOf("awaiting_flip")){ 
            set<bool>("facing_down", !get<bool>("facing_down"));  
        }  

        RobotState::manuallyModifyState("awaiting_claw_act", false); 
        RobotState::manuallyModifyState("awaiting_flip", false); 
    } 

}
 
void Claw::clench(bool clenched){ 
    return;
    //claw.set(!clenched);
} 

void Claw::flip(bool facingDown){   
    return;
    /*
    if (RobotState::getStateOf("inverted")){ 
      facingDown = !facingDown;
    } 
    wrist.set(facingDown); 
    */
}