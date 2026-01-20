#include "trajectorycontroller.h" 
#include "vex.h" 

double TrajectoryController::calculateFFOutput(){ 
    return feedForward->calculate(setpointVel, setpointAcc);
};

double TrajectoryController::calculatePDOutput(double currentPosition){ 
    return controller->calculate(currentPosition - setpointPos, Brain.Timer.time()); 
}; 

double TrajectoryController::calculate(double currentPosition){ 
    return calculatePDOutput(currentPosition) + calculateFFOutput();
}; 

void TrajectoryController::refreshSetpoints(double time){ 
    TrapezoidalSetpoint setpoint = profile.generateSetpoint(time);  

    lastPos = setpointPos; 
    lastVel = setpointVel; 

    setpointPos = setpoint.position; 
    setpointVel = setpoint.velocity; 
    setpointAcc = setpoint.acceleration;  
}; 

bool TrajectoryController::atGoal(){ 
    return profile.atGoal(lastPos, controller->getDerivative());
} 

void TrajectoryController::init(){ 
    controller->setLastTimestamp(Brain.Timer.time()); 
}
