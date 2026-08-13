#include "supersystem.h" 

double SuperSystem::BACKUP_DISTANCE = 100;

void SuperSystem::init(){ 
    set<int>("pickup_position", SuperStructurePosition::GROUND); 
    set<int>("position", get<int>("pickup_position"));   
    set<int>("last_position", get<int>("position"));  

    set<bool>("can_transition", true); 
    set<double>("distance_backed", 0);
}  

void SuperSystem::resetTaskProgress(){ 
    Telemetry::inst.placeValueAt<int>(0, "forearm","task_id"); 
}

void SuperSystem::refreshData(){    

    set<bool>("setpoints_reached",  
        Telemetry::inst.getValueAt<bool>("elevator", "at_setpoint") &&  
        Telemetry::inst.getValueAt<bool>("forearm", "at_setpoint"));   
    
    if (!get<bool>("can_transition")){ 
      double interval = 2; //Backwards velocity (fetched from odometry subtable)
      set<double>("distance_backed", get<double>("distance_backed") + interval);  

      if (get<double>("distance_backed") > BACKUP_DISTANCE){ 
        set<bool>("can_transition", true);  
        set<double>("distance_backed", 0);
      } 
    }
    
    if (get<bool>("setpoints_reached") && get<bool>("can_transition")){    
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
                  set<int>("position", get<int>("pickup_position"));
                }
                break; 
            case AUTO:  
                if (get<bool>("task_completed")){  //Must have another indicator that we are okay to drop (Back up to a certain extent)
                  set<int>("position", get<int>("pickup_position")); 
                  set<bool>("can_transition", false);
                }  
                break; 
            default: 
                break;
            }  
            set<bool>("macro_requested", false);
    }   

        

}
    
    
