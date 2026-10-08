#include "supersystem.h" 

const double SuperSystem::MINIMUM_SCORING_CLEAREANCE = 100; 

void SuperSystem::setPosition(int pos){  
    if (get<int>("position") != pos){ 
     switch (pos){
      case GROUND:
        if (get<int>("position") != SuperStructurePosition::PRIMED){ 
          Telemetry::inst.placeValueAt<bool>(true, "elevator", "hold");
          Telemetry::inst.placeValueAt<bool>(true, "forearm", "hold");
        }
        set<double>("transition_delay", 0);
        break;
      case PRIMED:  
        if (!(get<int>("position") == SuperStructurePosition::STANDING && Telemetry::inst.getValueAt<bool>("claw", "in_possession"))){ 
           if (get<int>("position") != SuperStructurePosition::GROUND || Telemetry::inst.getValueAt<bool>("claw", "in_possession")){ 
             Telemetry::inst.placeValueAt<bool>(true, "elevator", "hold");
             Telemetry::inst.placeValueAt<bool>(true, "forearm", "hold");
           }
        }
        set<double>("transition_delay", 0);
        break;
      case STANDING:
        Telemetry::inst.placeValueAt<bool>(true, "elevator", "hold");
        Telemetry::inst.placeValueAt<bool>(true, "forearm", "hold");
        set<double>("transition_delay", 0);
        break;
     }    
    }
    set<double>("transition_stamp", Brain.Timer.time()); 
    set<int>("position", pos);
} 

void SuperSystem::init(){  
    set<int>("position", SuperStructurePosition::PRIMED);
    setPosition(SuperStructurePosition::PRIMED);
}

void SuperSystem::refreshData(){   

    set<bool>("setpoints_reached",  
        Telemetry::inst.getValueAt<bool>("elevator", "at_setpoint") &&  
        Telemetry::inst.getValueAt<bool>("forearm", "at_setpoint"));    
    
    if (get<bool>("override") && get<int>("position") != SuperStructurePosition::EXIT){ 
      Telemetry::inst.placeValueAt<bool>(true, "elevator", "requesting_setpoint"); 
      Telemetry::inst.placeValueAt<bool>(true, "forearm", "requesting_setpoint");  
      Telemetry::inst.placeValueAt<double>(90, "forearm", "requested_setpoint"); 
      Telemetry::inst.placeValueAt<double>(750, "elevator", "requested_setpoint"); 
      set<int>("position", SuperStructurePosition::EXIT);
      set<bool>("override", false); 
    } else if (get<bool>("setpoints_reached")){
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
                  setPosition(SuperStructurePosition::AUTO); //Set the position to AUTO (essentially undefined
                  clearance = 0;
                } else if (!Telemetry::inst.getValueAt<bool>("claw", "in_possession")){ 
                   if (RobotState::getStateOf("grounded")){
                    setPosition(SuperStructurePosition::GROUND);
                   } else if (RobotState::getStateOf("standing")){ 
                    setPosition(SuperStructurePosition::STANDING);
                   }
                } 
                break;
            case AUTO:
                if (get<bool>("task_completed") && clearance > MINIMUM_SCORING_CLEAREANCE){ 
                  setPosition(SuperStructurePosition::PRIMED); 
                } else {
                  clearance += fabs(Telemetry::inst.getValueAt<double>("odometry", "immediate_distance"));  
                  Brain.Screen.printAt(20, 120, "Clearance: %.2f", clearance);
                }
                break;  
            case EXIT:  
                if (!RobotState::getStateOf("override")){ 
                   setPosition(SuperStructurePosition::PRIMED);
                }  
                break;
            default: 
                break;
            }  
            set<bool>("macro_requested", false); 
          
    }    
        
}