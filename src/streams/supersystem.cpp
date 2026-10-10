#include "supersystem.h" 

const double SuperSystem::MINIMUM_SCORING_CLEAREANCE = 100; 

void SuperSystem::setPosition(int pos){   
    //set<double>("transition_delay", 0); 
    set<double>("transition_stamp", Brain.Timer.time()); 
    set<int>("position", pos);
} 

void SuperSystem::init(){  
    setPosition(SuperStructurePosition::PRIMED);
}

void SuperSystem::refreshData(){   

    set<bool>("setpoints_reached",  
        Telemetry::inst.getValueAt<bool>("elevator", "at_setpoint") &&  
        Telemetry::inst.getValueAt<bool>("forearm", "at_setpoint"));    
    
    if (RobotState::getStateOf("override") && get<int>("position") != SuperStructurePosition::EXIT){  
      set<int>("position", SuperStructurePosition::EXIT);
    } else /*if (get<bool>("setpoints_reached"))*/{
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
                  Telemetry::inst.placeValueAt<bool>(true, "forearm", "active"); //First subsystem to act
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