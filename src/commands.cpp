#include "commands.h" 

void ModifyRobotState::start(){ 
    return;
} 

void ModifyRobotState::periodic(){ 
    Telemetry::inst.placeValueAt<bool>(entryVal, modDirectory, modName); 
    ran = true;
} 

void ModifyRobotState::end(){ 
    return;
} 

bool ModifyRobotState::isOver(){ 
    return ran;
}

//-------------------------------------------------------------------------- 

void WaitUntil::start(){ 
    return;
}

void WaitUntil::periodic(){ 
    return;
}

void WaitUntil::end(){ 
    return;
} 

bool WaitUntil::isOver(){ 
    return Telemetry::inst.getValueAt<bool>(directory, name) == desiredBool; 
}

//--------------------------------------------------------------------------

void WaitFor::start(){ 
    startingTimestamp = Brain.Timer.time(); 
} 

void WaitFor::periodic(){ 
    return; 
}

bool WaitFor::isOver(){ 
    return Brain.Timer.time() - startingTimestamp > duration;
} 

void WaitFor::end(){ 
    return; 
} 

//-------------------------------------------------------------------------------------- 

void WaitForSSState::start(){ 
    return;
} 

void WaitForSSState::periodic(){ 
    return; 
}

bool WaitForSSState::isOver(){ 
    return Telemetry::inst.getValueAt<int>("ss_manager", "position") == state;
} 

void WaitForSSState::end(){ 
    return; 
}  

//-------------------------------------------------------------------------------------- 

void ResetPose::start(){  
   Telemetry::inst.placeValueAt<bool>(true, "odometry", "dr_reset_triggered");
   for (ResetFace face : faces){ 
      if (face == ResetFace::FRONT){ 
         Telemetry::inst.placeValueAt<bool>(true, "odometry", "factor_front");
      } else if (face == ResetFace::LEFT){ 
         Telemetry::inst.placeValueAt<bool>(true, "odometry", "factor_left");
      } else if (face == ResetFace::BACK){ 
         Telemetry::inst.placeValueAt<bool>(true, "odometry", "factor_back");
      } else { 
         Telemetry::inst.placeValueAt<bool>(true, "odometry", "factor_right");
      }
   }
}

void ResetPose::periodic(){ 
    return;
}

bool ResetPose::isOver(){ 
    return !Telemetry::inst.getValueAt<bool>("odometry", "dr_reset_triggered");
}

void ResetPose::end(){ 
    return;
}


//--------------------------------------------------------------------------------------  

void ResetFromGoal::start(){  
   startTime = Brain.Timer.time();
}

void ResetFromGoal::periodic(){ 
    drivebaseRef.manualDrive(2,0); //A bit of meshing (but not a significnt amount) 
    if (Brain.Timer.time() - startTime >= 200 && !Telemetry::inst.getValueAt<bool>("odometry", "proxy_reset_triggered")){ 
      Telemetry::inst.placeValueAt<bool>(true, "odometry", "proxy_reset_triggered");
      Telemetry::inst.placeValueAt<int>(g, "odometry", "goal_index");  
      switched = true;
    }
}

bool ResetFromGoal::isOver(){ 
    return !Telemetry::inst.getValueAt<bool>("odometry", "proxy_reset_triggered") && switched;
}

void ResetFromGoal::end(){ 
    return;
}

//-------------------------------------------------------------------------------------- 


CommandInterface* Score(){ 
    return RaceCommandGroup::makeGroup( 
              { 
                SequentialCommandGroup::makeGroup( 
                 { 
                  ModifyRobotState::getCommand("ss_manager", "macro_requested", true), 
                  WaitUntil::getCommand("ss_manager", "task_completed", true)  
                 } 
                ),
                //WaitForSSState::getCommand(SuperStructurePosition::MATCHLOADING)
              }
           );
}   

CommandInterface* WaitForToggle(){  
    return SequentialCommandGroup::makeGroup({ 
             ModifyRobotState::getCommand("toggle", "auto_spin_enabled", true),
             WaitUntil::getCommand("toggle", "must_flip", false),
             ModifyRobotState::getCommand("toggle", "auto_spin_enabled", false)
           }); 
} 

CommandInterface* DriveIntoElement(CommandInterface* driveComm){ 
    return SequentialCommandGroup::makeGroup( 
        { 
           ModifyRobotState::getCommand("claw", "auto_clamp_enabled", true),
           RaceCommandGroup::makeGroup( 
            { 
             driveComm, 
             WaitForSSState::getCommand(SuperStructurePosition::PRIMED)
            }
           ),
           ModifyRobotState::getCommand("claw", "auto_clamp_enabled", false)
        }
    );
} 

CommandInterface* DriveIntoGoal(CommandInterface* driveComm){ 
    return RaceCommandGroup::makeGroup( 
        { 
            driveComm, 
            WaitUntil::getCommand("odometry", "obstructed", true)
        }
    );
}

CommandInterface* SnapHeading(){ 
    return SequentialCommandGroup::makeGroup( 
        { 
          ModifyRobotState::getCommand("odometry", "ang_reset_triggered", true), 
          WaitUntil::getCommand("odometry", "ang_reset_triggered", false)
        }
    );
}

CommandInterface* Clench(){  
    return SequentialCommandGroup::makeGroup( 
        { 
          ModifyRobotState::getCommand("claw", "auto_clamp_enabled", true), 
          RaceCommandGroup::makeGroup( 
            { 
               WaitFor::getCommand(300), 
               WaitUntil::getCommand("claw", "clamped", true)
            }
          ), 
          ModifyRobotState::getCommand("claw", "auto_clamp_enabled", false) 
        }
    ); 
}

vector<CommandInterface*> RoutineWrap(vector<CommandInterface*> content){  
  content.insert( 
     content.begin(),   
     WaitUntil::getCommand("odometry", "opened", true)
  );
  return { 
     ParallelCommandGroup::makeGroup({
        SequentialCommandGroup::makeGroup( 
            { 
                content 
            }) 
     })->addBackgroundTasks(
        { 
         RunSuperStructure::getCommand()
        } 
     )
  }; 
};





