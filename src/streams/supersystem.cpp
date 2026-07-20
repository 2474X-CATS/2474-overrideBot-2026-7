#include "supersystem.h" 

void SuperSystem::init(){ 
    set<int>("position", SuperStructurePosition::PRIMED); 
}  

void SuperSystem::resetTaskProgress(){ 
    Telemetry::inst.placeValueAt<int>(0, "forearm","task_id"); 
}

void SuperSystem::refreshData(){       
    string stateMessage; 
    switch (get<int>("position")){ 
     case AUTO:  
       stateMessage = "AUTO";  
       break;
     case PRIMED: 
       stateMessage = "PRIMED";  
       break;
     case GROUND: 
       stateMessage = "GROUND";  
       break;
     case STANDING: 
       stateMessage = "STANDING";  
       break;
    } 
    Brain.Screen.printAt(20, 160, stateMessage.c_str()); 
    Brain.Screen.printAt(20, 190, "Setpoints Reached: %d", Telemetry::inst.getValueAt<bool>("ss_manager", "setpoints_reached"));
    

    set<bool>("setpoints_reached",  
        Telemetry::inst.getValueAt<bool>("elevator", "at_setpoint") &&  
        Telemetry::inst.getValueAt<bool>("forearm", "at_setpoint"));  
    
    if (get<bool>("in_autopilot")){ 
        if (get<bool>("task_completed")){ 
            set<int>("position", SuperStructurePosition::GROUND);  
            set<bool>("in_autopilot", false); 
        }
    } else { 
        if (get<bool>("macro_requested")){  
            set<bool>("macro_requested", false);
            if (get<bool>("setpoints_reached") && get<int>("position") == SuperStructurePosition::PRIMED){ 
               Telemetry::inst.placeValueAt<bool>(true, "elevator", "active"); //First subsystem to act
               set<bool>("task_completed", false); //Task is not completed
               set<bool>("in_autopilot", true); //Officially running the macro
               set<int>("position", SuperStructurePosition::AUTO); //Set the position to AUTO (essentially undefined)
               resetTaskProgress();
               return;
            }
        } 
        if (get<bool>("matchloader_requested")){ 
            set<bool>("matchloader_requested", false);   
            if (get<bool>("setpoints_reached") && get<int>("position") == SuperStructurePosition::GROUND){ 
               set<int>("position", SuperStructurePosition::STANDING); 
               return;
            }
        } 

        if (get<bool>("setpoints_reached")){   
            switch (get<int>("position")){ 
                case GROUND:  
                  if (Telemetry::inst.getValueAt<bool>("claw","clenched")){ 
                    set<int>("position", SuperStructurePosition::PRIMED);
                  } 
                  break;  
                case STANDING: 
                  if (Telemetry::inst.getValueAt<bool>("claw","clenched")){ 
                    set<int>("position", SuperStructurePosition::PRIMED);
                  }  
                  break; 
                case PRIMED:  
                  if (!Telemetry::inst.getValueAt<bool>("claw","clenched")){ 
                    set<int>("position", SuperStructurePosition::GROUND);
                  }
                  break;   
            }
        } 
       
        Telemetry::inst.placeValueAt<bool>(false, "elevator", "responded"); 
        Telemetry::inst.placeValueAt<bool>(false, "forearm", "responded"); 
    }
    
    
}