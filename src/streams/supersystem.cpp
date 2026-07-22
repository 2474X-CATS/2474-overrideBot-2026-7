#include "supersystem.h" 

void SuperSystem::init(){ 
    set<int>("pickup_position", SuperStructurePosition::GROUND); 
    set<int>("position", get<int>("pickup_position"));   
}  

void SuperSystem::resetTaskProgress(){ 
    Telemetry::inst.placeValueAt<int>(0, "forearm","task_id"); 
}

void SuperSystem::refreshData(){    

    set<bool>("setpoints_reached",  
        Telemetry::inst.getValueAt<bool>("elevator", "at_setpoint") &&  
        Telemetry::inst.getValueAt<bool>("forearm", "at_setpoint"));  
    
    if (get<bool>("setpoints_reached")){    
        bool inPossession = Telemetry::inst.getValueAt<bool>("claw","clenched") && Telemetry::inst.getValueAt<bool>("claw","senses_object");
        switch (get<int>("position")){ 
            case GROUND:
                if (inPossession){ 
                  set<int>("position", SuperStructurePosition::PRIMED);
                } else { 
                  if (get<int>("pickup_position") == SuperStructurePosition::STANDING){ 
                    set<int>("position", SuperStructurePosition::STANDING);
                  }
                }
                break;  
            case STANDING: 
                if (inPossession){ 
                  set<int>("position", SuperStructurePosition::PRIMED);
                } else { 
                  if (get<int>("pickup_position") == SuperStructurePosition::GROUND){ 
                    set<int>("position", SuperStructurePosition::GROUND);
                  }
                }
                break; 
            case PRIMED:   
                if (get<bool>("macro_requested")){ 
                  Telemetry::inst.placeValueAt<bool>(true, "elevator", "active"); //First subsystem to act
                  set<bool>("task_completed", false); //Task is not completed
                  set<int>("position", SuperStructurePosition::AUTO); //Set the position to AUTO (essentially undefined)
                  resetTaskProgress();
                } else if (!inPossession){ 
                  set<int>("position", SuperStructurePosition::GROUND);
                }
                break; 
            case AUTO: 
                if (get<bool>("task_completed")){ 
                  set<int>("position", SuperStructurePosition::GROUND);  
                } 
                break; 
            default: 
                break;
            }  
            set<bool>("macro_requested", false);
    }   
}
    
    