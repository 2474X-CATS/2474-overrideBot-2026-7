#include "supersystem.h" 
//double SuperSystem::BACKUP_DISTANCE = 100;

void SuperSystem::setPosition(int pos){ 
    switch (pos){ 
      case GROUND:  
        Telemetry::inst.placeValueAt<bool>(true, "elevator", "hold"); 
        Telemetry::inst.placeValueAt<bool>(true, "forearm", "hold");
        break;
      case STANDING:
        Telemetry::inst.placeValueAt<bool>(true, "elevator", "hold"); 
        break; 
      case PRIMED:
        break;
      default:
        break;
    } 
    set<int>("position", pos);
} 

void SuperSystem::init(){ 
    setPosition(SuperStructurePosition::PRIMED);
}  

void SuperSystem::refreshData(){   

    set<bool>("setpoints_reached",  
        Telemetry::inst.getValueAt<bool>("elevator", "at_setpoint") &&  
        Telemetry::inst.getValueAt<bool>("forearm", "at_setpoint"));   
      
    
    if (get<bool>("setpoints_reached")){
        switch (get<int>("position")){ 
            case GROUND:
                if (!RobotState::getStateOf("grounded")){  
                  if (RobotState::getStateOf("standing")){ 
                    setPosition(SuperStructurePosition::STANDING); 
                  } else {
                    setPosition(SuperStructurePosition::PRIMED); 
                  }
                }
                break;
            case STANDING: 
                if (RobotState::getStateOf("in_autonomous") && !Telemetry::inst.getValueAt<bool>("claw", "waiting") && Telemetry::inst.getValueAt<bool>("claw", "in_possession")){ 
                  RobotState::manuallyModifyState("standing", false);
                }
                if (!RobotState::getStateOf("standing")){ 
                  if (RobotState::getStateOf("grounded")){
                    setPosition(SuperStructurePosition::GROUND); 
                  } else { 
                    setPosition(SuperStructurePosition::PRIMED); 
                  }
                } 
                break;
            case PRIMED:
                if (get<bool>("macro_requested") && Telemetry::inst.getValueAt<bool>("claw", "in_possession")){ 
                  Telemetry::inst.placeValueAt<bool>(true, "elevator", "active"); //First subsystem to act
                  set<bool>("task_completed", false); //Task is not completed
                  setPosition(SuperStructurePosition::AUTO); //Set the position to AUTO (essentially undefined)
                } else if (!Telemetry::inst.getValueAt<bool>("claw", "in_possession") && RobotState::getStateOf("standing")){ 
                  setPosition(SuperStructurePosition::STANDING);
                } else if (RobotState::getStateOf("grounded")){ 
                  setPosition(SuperStructurePosition::GROUND); 
                }
                break;
            case AUTO:
                if (get<bool>("task_completed")){  //Must have another indicator that we are okay to drop (Back up to a certain extent)
                  setPosition(SuperStructurePosition::PRIMED); 
                }
                break; 
            default: 
                break;
            }  
            set<bool>("macro_requested", false);
    }    
        
}