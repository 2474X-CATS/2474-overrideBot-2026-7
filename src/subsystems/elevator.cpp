#include "elevator.h"  

Elevator* Elevator::globalPtr = nullptr;

 
double Elevator::LEVELED_HEIGHT = (17.678 + 2.75) * 25.4; 
double Elevator::GROUND_INTAKE_HEIGHT = LEVELED_HEIGHT + 40;  

double Elevator::MAX_HEIGHT = (42 * 25.4);

double Elevator::ELEVATOR_ERROR_TOLERANCE = 3; 
double Elevator::STACK_HEIGHT = 100;

double Elevator::PRIMING_SPEED = 12; 

double Elevator::MINIMUM_ALIGNER_DISTANCE = 0.0;  
double Elevator::ALIGNER_ERROR_TOLERANCE = 0.0;  

double Elevator::SPOOL_DIAMETER = (Elevator::MAX_HEIGHT - Elevator::LEVELED_HEIGHT) / (2.534 * M_PI);

Elevator& Elevator::getObject(){ 
  return *globalPtr;
}

void Elevator::init(){ 
    //Set up all the constants   

    //Feedback: Correcting the Feedforward's shortcomings 

    PIDConstants pidConsts;
    pidConsts.P = 0.5;
    pidConsts.I = 0.0;
    pidConsts.D = 0.001;
    pidConsts.errorTolerance = 10;  

    lift.setStopping(vex::brakeType::brake); 
  
    rot.setPosition(0, vex::rotationUnits::rev);
   
    currentState = ElevatorState::PRIMING;

    correctionController = new pidcontroller(pidConsts, getPosition());   
    correctionController->setLastTimestamp(Brain.Timer.time());

} 

void Elevator::periodic(){    
   double elevatorOutput; 
   if (currentState == ElevatorState::HOLDING || currentState == ElevatorState::PURSUING){ //
     elevatorOutput = correctionController->calculate(getPosition(), Brain.Timer.time()); 
   } else if (currentState == ElevatorState::PRIMING){ 
     elevatorOutput = PRIMING_SPEED;
   } else if (currentState == ElevatorState::LISTLESS) { 
     elevatorOutput = 0;
   } else {  
     elevatorOutput = PRIMING_SPEED * raisingDirection;
   }
   lift.spin(vex::directionType::rev, elevatorOutput, vex::voltageUnits::volt);
}  

void Elevator::updateTelemetry(){     
    // Update status of stack sight    

    double currentPosition = getPosition(); 
    double currentVelocity = getVelocity();  

    set<bool>("sensing_stack", currentPosition < 750);//fabs(primingSensor.objectDistance(vex::distanceUnits::mm) - MINIMUM_ALIGNER_DISTANCE) < ALIGNER_ERROR_TOLERANCE);
    set<double>("current_velocity", currentVelocity);      
    set<double>("current_height", currentPosition); 
    set<double>("percentage_extended", (currentPosition - LEVELED_HEIGHT) / (MAX_HEIGHT - LEVELED_HEIGHT)); 
    stateControl();
    if (!RobotState::getStateOf("in_autonomous")){ 
       respondToRequests();
    } 
    //Brain.Screen.printAt(20, 120, "Current position: %.2f", getPosition()); 
    //Brain.Screen.printAt(20, 140, "Current pos(rotations): %.2f", rot.position(vex::rotationUnits::rev)); 
} 

double Elevator::getPosition(){ 
    return (rot.position(vex::rotationUnits::rev) * M_PI * SPOOL_DIAMETER) + LEVELED_HEIGHT;
} 

double Elevator::getVelocity(){
   return rot.velocity(vex::velocityUnits::rpm) / 60 * M_PI * SPOOL_DIAMETER; //(getPosition() - previousHeight) / ((Brain.Timer.time() - previousTimestamp) / 1000.0); 
}

void Elevator::setSetpoint(double setpoint){  
   correctionController->setSetpoint(setpoint);
   correctionController->setLastTimestamp(Brain.Timer.time());
   currentState = ElevatorState::PURSUING;
}

bool Elevator::reachedSetpoint(){ 
   return correctionController->atSetpoint(getPosition()); //Brain.Timer.time() - motionProfile->getStartTime() >= motionProfile->getTotalDuration(); 
} 

void Elevator::lock(){
  correctionController->reset();
  correctionController->setSetpoint(getPosition());  
  correctionController->setLastTimestamp(Brain.Timer.time()); 
}

void Elevator::stateControl(){  

    if (get<bool>("requesting_setpoint")){   
      setSetpoint(get<double>("requested_height"));
      set<bool>("requesting_setpoint", false); 
    }

    if (currentState == ElevatorState::PURSUING && reachedSetpoint()){ //
        currentState = ElevatorState::HOLDING;
    } else if (currentState == ElevatorState::PRIMING && (!get<bool>("sensing_stack"))){   
        lock();
        currentState = ElevatorState::HOLDING; //Reached top of stack 
    }

    set<bool>("at_setpoint", currentState != ElevatorState::PURSUING && currentState != ElevatorState::PRIMING); 
    
}

void Elevator::respondToRequests(){     
   if (RobotState::getStateOf("rise")){ 
      raisingDirection = 1;
   } else if (RobotState::getStateOf("fall")){ 
      raisingDirection = -1;
   } else { 
      raisingDirection = 0;
   } 

   if (currentState == ElevatorState::PURSUING){ 
    return;
   }

   if (RobotState::getStateOf("awaiting_land")){  
     set<bool>("requesting_setpoint", true);
     set<double>("requested_height", GROUND_INTAKE_HEIGHT); 
     return;
   }

   if (raisingDirection == 0){ 
     if (currentState == ElevatorState::ADJUSTING){
       currentState = ElevatorState::LISTLESS;
     }
   } else { 
     currentState = ElevatorState::ADJUSTING;
   }
   
} 

void Elevator::stop(){ 
  return;
}