#include "elevator.h" 

Elevator* Elevator::globalPtr = nullptr;

double Elevator::LEVELED_HEIGHT = (17.678 + 2.75) * 25.4; 
double Elevator::GROUND_INTAKE_HEIGHT = LEVELED_HEIGHT + 40;  
       
double Elevator::MAX_HEIGHT = (42 * 25.4);

//double Elevator::ELEVATOR_ERROR_TOLERANCE = 3; 
double Elevator::STACK_HEIGHT = 100;

double Elevator::PRIMING_SPEED = 12;

double Elevator::MINIMUM_ALIGNER_DISTANCE = 0.0;  
double Elevator::ALIGNER_ERROR_TOLERANCE = 0.0;  

double Elevator::SPOOL_DIAMETER = (Elevator::MAX_HEIGHT - Elevator::LEVELED_HEIGHT) / (2.534 * M_PI);

Elevator& Elevator::getObject(){ 
  return *globalPtr;
}

void Elevator::init(){  

    PIDConstants pidConsts;  
    pidConsts.P = 0.525;
    pidConsts.I = 0.001;
    pidConsts.D = 0; 
    pidConsts.errorTolerance = 10;

    lift.setStopping(vex::brakeType::brake); 
    rot.setPosition(0, vex::rotationUnits::rev);  

    correctionController = new pidcontroller(pidConsts, getPosition());
    correctionController->setLastTimestamp(Brain.Timer.time()); 

} 

void Elevator::stop(){ 
   lift.spin(vex::directionType::fwd, 0, vex::voltageUnits::volt);
}

void Elevator::periodic(){
   double elevatorOutput;
   if (currentState == ElevatorState::E_HOLDING || currentState == ElevatorState::E_PURSUING){//Stay Still
     elevatorOutput = correctionController->calculate(getPosition(), Brain.Timer.time()); 
   } else if (currentState == ElevatorState::E_PRIMING){ //Rise or fall at a constant rate
     elevatorOutput = PRIMING_SPEED;
   } else if (currentState == ElevatorState::E_ADJUSTING){ 
     elevatorOutput = PRIMING_SPEED * raisingDirection;
   } else {  //Pursuing a setpoint
     elevatorOutput = 0;
   } 
   lift.spin(vex::directionType::rev, elevatorOutput, vex::voltageUnits::volt);
}  

void Elevator::updateTelemetry(){     
    // Update status of stack sight   
    set<bool>("sensing_stack", getPosition() < 750);//fabs(primingSensor.objectDistance(vex::distanceUnits::mm) - MINIMUM_ALIGNER_DISTANCE) < ALIGNER_ERROR_TOLERANCE);
    set<double>("percentage_extended", (getPosition() - LEVELED_HEIGHT) / (MAX_HEIGHT - LEVELED_HEIGHT));
    
    stateControl();
    if (!RobotState::getStateOf("in_autonomous")){ 
       respondToRequests();
    } else { 
       raisingDirection = 0;
    } 
} 

double Elevator::getPosition(){ 
    return (rot.position(vex::rotationUnits::rev) * M_PI * SPOOL_DIAMETER) + LEVELED_HEIGHT;
} 

double Elevator::getVelocity(){
   return rot.velocity(vex::velocityUnits::rpm) / 60 * M_PI * SPOOL_DIAMETER; 
}

void Elevator::setSetpoint(double setpoint){   
   correctionController->setSetpoint(setpoint); 
   correctionController->setLastTimestamp(Brain.Timer.time()); 
   currentState = ElevatorState::E_PURSUING;
}

bool Elevator::reachedSetpoint(){ 
   return correctionController->atSetpoint(getPosition());  
}
 
void Elevator::lock(){ 
   correctionController->setSetpoint(getPosition());   
   correctionController->setLastTimestamp(Brain.Timer.time()); 
} 

void Elevator::stateControl(){ 
    
    SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));   

    if (requestingSetpoint){  
      if (get<bool>("sniper_score_enabled")){ 
        primingSetpoint = requestedHeight;
      } else { 
        setSetpoint(requestedHeight); 
      }
      requestingSetpoint = false;
    }

    if (currentState == ElevatorState::E_PURSUING){ //
      if (reachedSetpoint()){  
        currentState = ElevatorState::E_HOLDING; 
      }
    } else if (currentState == ElevatorState::E_PRIMING){   
        if (!get<bool>("sensing_stack")){ 
          currentState = ElevatorState::E_HOLDING;  
          lock();
          set<bool>("active", false); 
          Telemetry::inst.placeValueAt<bool>(true, "forearm", "active");
        }
    } else {  
        switch (pos){
            case AUTO:
               if (get<bool>("active")){   
                 if (get<bool>("sniper_score_enabled")){ 
                   setSetpoint(primingSetpoint);
                 } else { 
                   currentState = ElevatorState::E_PRIMING; 
                 }
                 set<bool>("sniper_score_enabled", false);
               }
               break; 
            case GROUND:  
               requestingSetpoint = true;
               requestedHeight = GROUND_INTAKE_HEIGHT;  
               break; 
            case STANDING: 
               requestingSetpoint = true; 
               requestedHeight = LEVELED_HEIGHT; 
               break;
            case PRIMED:
               if (get<bool>("sniper_score_enabled")){ 
                  setSetpoint(primingSetpoint); 
                  set<bool>("sniper_score_enabled", false);
               } 
               break;
            default:
               break;
        }
    }   

   set<bool>("at_setpoint", currentState != ElevatorState::E_PURSUING && currentState != ElevatorState::E_PRIMING);   
}

void Elevator::respondToRequests(){   
    if (currentState == E_PRIMING || currentState == E_PURSUING){ 
      return;
    } 

    SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));
    
    if (pos == SuperStructurePosition::PRIMED && Telemetry::inst.getValueAt<bool>("ss_manager", "setpoints_reached")){   
      if (RobotState::getStateOf("awaiting_land")){ 
        requestingSetpoint = true; 
        requestedHeight = GROUND_INTAKE_HEIGHT; 
        return;
      }  
      if (RobotState::getStateOf("rise")){ 
        currentState = E_ADJUSTING; 
        raisingDirection = 1;
      } else if (RobotState::getStateOf("fall")){ 
        raisingDirection = -1; 
        currentState = E_ADJUSTING; 
      } else {   
        currentState = E_LISTLESS;
        raisingDirection = 0;
      }
    } else { 
      raisingDirection = 0;
    }
    /*
      if (get<int>("priming_direction") != 0){ 
        if ((Brain.Timer.time() - get<double>("lifting_timestamp")) >= 100){ //Wait tenth of a second  
          int stacks = (get<double>("current_height") / (STACK_HEIGHT - 20)); // 1 cm of grace
          stacks += copysign(1, get<int>("priming_direction"));  
          if (stacks * STACK_HEIGHT <= MAX_HEIGHT && stacks >= 0){ 
            set<bool>("requesting_setpoint", true);  
            set<double>("requested_height", stacks * 1.0 * STACK_HEIGHT); 
          }   
          set<double>("lifting_timestamp", Brain.Timer.time());
          raisingDirection = 0;
        }
      } else { 
        set<double>("lifting_timestamp", Brain.Timer.time());  
      }   
      */ 
} 

//--------------------------------------- 

void RunElevator::start(){ 
  return;
} 

void RunElevator::periodic(){ 
  elevatorRef.periodic();
} 

bool RunElevator::isOver(){ 
  return false;
} 

void RunElevator::end(){ 
  return;
}