#include "commands.h" 


void ModifyRobotState::start(){ 
    return;
} 

void ModifyRobotState::periodic(){ 
    Telemetry::inst.placeValueAt<bool>(entryVal, modDirectory, modName);
} 

void ModifyRobotState::end(){ 
    return;
} 

bool ModifyRobotState::isOver(){ 
    return true;
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
