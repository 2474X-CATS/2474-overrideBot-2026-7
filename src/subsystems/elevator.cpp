#include "elevator.h"  

Elevator* Elevator::globalPtr = nullptr;

double Elevator::GROUND_INTAKE_HEIGHT = 100;  
double Elevator::LEVELED_HEIGHT = 0; 
       
double Elevator::MAX_HEIGHT = 1000;

double Elevator::ELEVATOR_ERROR_TOLERANCE = 3; 
double Elevator::STACK_HEIGHT = 0.0;  

double Elevator::PRIMING_SPEED = 500; 

double Elevator::MINIMUM_ALIGNER_DISTANCE = 0.0;  
double Elevator::ALIGNER_ERROR_TOLERANCE = 0.0; 

Elevator& Elevator::getObject(){ 
  return *globalPtr;
}

void Elevator::init(){ 
    //Set up all the constants   

    //Motion: Max speed and acceleration
     
    motionConsts.maxVelocity = 0; 
    motionConsts.maxAcceleration = 0; 
  
    //Feedback: Correcting the Feedforward's shortcomings 

    PIDConstants pidConsts;  
    pidConsts.P = 0; 
    pidConsts.I = 0; 
    pidConsts.D = 0; 

    correctionController = new pidcontroller(pidConsts, 0);   

    //Feedforward: Bulk of precision contol based on an inverse model of the elevator system 

    elevatorFF.ffConsts.kS = 0; 
    elevatorFF.ffConsts.kV = 0;
    elevatorFF.ffConsts.kA = 0;  
    elevatorFF.kG = 0;

    correctionController->setLastTimestamp(Brain.Timer.time());
   
} 

void Elevator::periodic(){   
   double elevatorOutput;
   if (currentState == ElevatorState::HOLDING){//Stay Still
     elevatorOutput = calculateOutput(PRIMING_SPEED * raisingDirection, 0); 
   } else if (currentState == ElevatorState::PRIMING){ //Rise or fall at a constant rate
     elevatorOutput = calculateOutput(PRIMING_SPEED, 0); 
   } else {  //Pursuing a setpoint
     TrapezoidalSetpoint motionGoal = motionProfile->generateSetpoint(Brain.Timer.time());
     elevatorOutput = calculateOutput(motionGoal.velocity, motionGoal.acceleration) * setpointDirection;
   } 
   lift.spin(vex::directionType::fwd, elevatorOutput, vex::voltageUnits::volt);
}  

void Elevator::updateTelemetry(){     
    // Update status of stack sight   
    set<bool>("sensing_stack", fabs(primingSensor.objectDistance(vex::distanceUnits::mm) - MINIMUM_ALIGNER_DISTANCE) < ALIGNER_ERROR_TOLERANCE);
    updatePosition(); 
    stateControl();
    if (!RobotState::getStateOf("in_autonomous")){ 
       respondToRequests();
    } else { 
       raisingDirection = 0;
    } 
} 

double Elevator::getPosition(){ 
    return 0.0; 
} 

double Elevator::getVelocity(){
   return (getPosition() - previousHeight) / ((Brain.Timer.time() - previousTimestamp) / 1000.0); 
}

void Elevator::updatePosition(){ 
  double newPosition = getPosition();   
  set<double>("current_velocity", getVelocity());   
  previousHeight = get<double>("current_height");   
  set<double>("current_height", newPosition);  
  previousTimestamp = Brain.Timer.time(); 
}

double Elevator::calculateOutput(double velocity, double acceleration){  
    double ffOutput = elevatorFF.calculate(velocity, acceleration); 
    double pidOutput = correctionController->calculate(getVelocity() - velocity, Brain.Timer.time()); 
    return ffOutput + pidOutput; 
} 

void Elevator::setSetpoint(double setpoint){ 
   double error = setpoint - getPosition();  
   setpointDirection = copysign(1, error); 
   error = fabs(error); 
   if (error < ELEVATOR_ERROR_TOLERANCE){ 
    return;
   } 
   motionProfile = new TrapezoidalMotionProfile(motionConsts, error, getVelocity(), 0);   
   motionProfile->setLastTimestamp(Brain.Timer.time()); 
   correctionController->setLastTimestamp(Brain.Timer.time());
   currentState = ElevatorState::PURSUING;
}

bool Elevator::reachedSetpoint(){ 
   return Brain.Timer.time() - motionProfile->getStartTime() >= motionProfile->getTotalDuration(); 
}

void Elevator::stateControl(){  

    if (get<bool>("requesting_setpoint")){   
      setSetpoint(get<double>("requested_height"));
      set<bool>("requesting_setpoint", false); 
    }

    if (currentState == ElevatorState::PURSUING){ //
      if (reachedSetpoint()){  
        currentState = ElevatorState::HOLDING; 
      }
    } else if (currentState == ElevatorState::PRIMING){   
        if (!get<bool>("sensing_stack")){ 
          currentState = ElevatorState::HOLDING; 
        }
    }

   set<bool>("at_setpoint", currentState == ElevatorState::HOLDING);
}

void Elevator::respondToRequests(){ 
    if (currentState == ElevatorState::HOLDING){ 
      raisingDirection = ((int)RobotState::getAxisState(AxisType::S_LEFT_VERTICAL)) / 100;  
      if (get<double>("current_height") >= MAX_HEIGHT){ 
        raisingDirection = min<int>(raisingDirection, 0);
      } else if (get<double>("current_height") <= GROUND_INTAKE_HEIGHT){ 
        raisingDirection = max<int>(raisingDirection, 0);
      }
    }
} 

void Elevator::stop(){ 
  return;
}