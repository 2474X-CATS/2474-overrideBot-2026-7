#include "elevator.h" 

void Elevator::init(){ 
    //Set up all the constants
} 

void Elevator::periodic(){ 
    lift.spin(vex::forward, calculateOutput(), vex::volt);  
} 

void Elevator::updateTelemetry(){   
    stateControl();
    if (!RobotState::getStateOf("in_autonomous")){ 
       respondToRequests();
    }
} 

double Elevator::calculateOutput(){ 
    return 0.0; 
    //Arm output depending on setpoints
} 

void Elevator::stateControl(){ 
    return; 
    //state management
}

void Elevator::respondToRequests(){ 
    return; 
    //respond to driver/operator input
}