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
    return Telemetry::inst.getValueAt<int>("ss_manager", "position") == state && Telemetry::inst.getValueAt<bool>("ss_manager", "setpoints_reached");
} 

void WaitForSSState::end(){ 
    return; 
} 

//--------------------------------------------------------------------------------------  


CommandInterface* Score(){ 
    return SequentialCommandGroup::makeGroup( 
      Prime(true)  
    )->chainThen( 
      ModifyRobotState::getCommand("ss_manager", "macro_requested", true)
    )->chainThen( 
      WaitForSSState::getCommand(SuperStructurePosition::PRIMED)
    );
}  

CommandInterface* GroundIntakeMode(bool waitUntilReached){ 
    SequentialCommandGroup* group = SequentialCommandGroup::makeGroup(ModifyRobotState::getCommand("robot_state", "standing", false)); 
    group->chainThen(ModifyRobotState::getCommand("robot_state", "grounded", true));
    if (waitUntilReached){ 
       group->chainThen(WaitForSSState::getCommand(SuperStructurePosition::GROUND));
    } 
    return group;
} 

CommandInterface* StandingMode(bool waitUntilReached){ 
    SequentialCommandGroup* group = SequentialCommandGroup::makeGroup(ModifyRobotState::getCommand("robot_state", "grounded", false)); 
    group->chainThen(ModifyRobotState::getCommand("robot_state", "standing", true));
    if (waitUntilReached){ 
       group->chainThen(WaitForSSState::getCommand(SuperStructurePosition::STANDING));
    }
    return group;
}   

CommandInterface* Prime(bool waitUntilReached){ 
    SequentialCommandGroup* group = SequentialCommandGroup::makeGroup(ModifyRobotState::getCommand("robot_state", "grounded", false));  
    group->chainThen(ModifyRobotState::getCommand("robot_state", "standing", false)); 
    if (waitUntilReached){ 
       group->chainThen(WaitForSSState::getCommand(SuperStructurePosition::PRIMED));
    }
    return group;
} 

CommandInterface* Drop(int timeout){ 
    SequentialCommandGroup* group = SequentialCommandGroup::makeGroup(ModifyRobotState::getCommand("robot_state", "purge", true));  
    group->chainThen(WaitFor::getCommand(timeout))-> 
    chainThen(ModifyRobotState::getCommand("robot_state", "purge", false));
    return group; 
}



