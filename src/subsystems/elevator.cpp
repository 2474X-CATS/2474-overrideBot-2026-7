#include "elevator.h" 


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

    correctionController = new errorcontroller(pidConsts);   

    //Feedforward: Bulk of precision contol based on an inverse model of the elevator system 

    elevatorFF.ffConsts.kS = 0; 
    elevatorFF.ffConsts.kV = 0;
    elevatorFF.ffConsts.kA = 0;  
    elevatorFF.kG = 0; 

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
    double pidOutput = correctionController->calculate(getVelocity(), Brain.Timer.time()); 
    correctionController->setReference(velocity); 
    return ffOutput + pidOutput; 
} 

void Elevator::setSetpoint(double setpoint){ 
   double error = setpoint - getPosition();  
   setpointDirection = copysign(1, error); 
   error = fabs(error); 
   if (error < ELEVATOR_ERROR_TOLERANCE || currentState != ElevatorState::HOLDING){ 
    return;
   } 
   motionProfile = new TrapezoidalMotionProfile(motionConsts, error, getVelocity(), 0);   
   motionProfile->setLastTimestamp(Brain.Timer.time());
   currentState = ElevatorState::PURSUING;
}

bool Elevator::reachedSetpoint(){ 
   return Brain.Timer.time() - motionProfile->getStartTime() >= motionProfile->getTotalDuration(); 
}

void Elevator::stateControl(){ 
    SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));   
    
    if (get<bool>("requesting_setpoint")){ 
      set<bool>("requesting_setpoint", false); 
      setSetpoint(get<double>("requested_height"));
    }

    if (currentState == ElevatorState::PURSUING){ //
      if (reachedSetpoint()){ 
        currentState = ElevatorState::HOLDING;
      }
    } else if (currentState == ElevatorState::PRIMING){ 
        if (!get<bool>("sensing_stack")){ 
          currentState = ElevatorState::HOLDING; 
          set<bool>("active", false); 
          Telemetry::inst.placeValueAt<bool>(true, "forearm", "active");
        }
    } else { 
        switch (pos){ 
            case AUTO: 
               if (get<bool>("active")){  
                 currentState = ElevatorState::PRIMING;
               } 
               break; 
            case GROUND:
               set<bool>("requesting_setpoint", true);
               set<double>("requested_height", GROUND_INTAKE_HEIGHT);  
               break; 
            case STANDING: 
               set<bool>("requesting_setpoint", true); 
               set<double>("requested_height", LEVELED_HEIGHT); 
               break; 
            default: 
               break;
        }
        
    }
    /*  
    - Needs to check if the elevator *has* a setpoint yet 
      [This is essentially checking if there is an important height the elevator needs 
      to reach. If there is a named position or a known height we want to reach then we will  
      have a setpoint. In this case the motion profile will not be a nullptr] 
    - If we don't have a known setpoint though then 
    | 
    v
    If in the GROUND position or STANDING position we will have a setpoint (specifically the ground position) 
    | 
    v 
    Because we want to go to specific heights in the autonomous period then we will have setpoints (but this
    can be controlled by an entry "requested_height") In fact ground and standing will also run the same  
    protocol. Modifying requested_height and has_setpoint to show that we want to go to a specifc height.  
    | 
    v  
    Finally (actually this should be at the start) when we are running the macro the first stage is to prime 
    at least when we aren't in auto. (Going to make a prime function)
    */   

   set<bool>("at_setpoint", currentState == ElevatorState::HOLDING);

}

void Elevator::respondToRequests(){ 
    SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));
    //respond to driver/operator input 
    if (pos == SuperStructurePosition::PRIMED && currentState == ElevatorState::HOLDING){ 
      raisingDirection = ((int)RobotState::getAxisState(AxisType::S_LEFT_VERTICAL)) / 100;  
      if (get<double>("current_height") >= MAX_HEIGHT){ 
        raisingDirection = min<int>(raisingDirection, 0);
      } else if (get<double>("current_height") <= GROUND_INTAKE_HEIGHT){ 
        raisingDirection = max<int>(raisingDirection, 0);
      }
    }
}