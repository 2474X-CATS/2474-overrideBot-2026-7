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


CommandInterface* Score(){ 
    SequentialCommandGroup* cmd = SequentialCommandGroup::makeGroup(ModifyRobotState::getCommand("ss_manager", "macro_requested", true))-> 
    chainThen(WaitUntil::getCommand("ss_manager", "task_completed", true));
    
    return RaceCommandGroup::makeGroup(cmd)->addRacer(WaitForSSState::getCommand(SuperStructurePosition::MATCHLOADING));
}   

CommandInterface* WaitForToggle(){ 
    return WaitUntil::getCommand("toggle", "must_flip", false); 
}  

CommandInterface* DriveIntoElement(CommandInterface* driveComm){ 
    RaceCommandGroup* cmd = RaceCommandGroup::makeGroup(driveComm); 
    cmd->addRacer(WaitForSSState::getCommand(SuperStructurePosition::PRIMED)); 
    return cmd;
} 

CommandInterface* DriveIntoGoal(CommandInterface* driveComm){ 
    RaceCommandGroup* cmd = RaceCommandGroup::makeGroup(driveComm); 
    cmd->addRacer(WaitUntil::getCommand("odometry", "obstructed", true)); 
    return cmd;
}



   



