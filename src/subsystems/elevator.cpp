#include "elevator.h" 
#include "../utilities/functools.h"

Elevator* Elevator::globalPtr = nullptr;

double Elevator::LEVELED_HEIGHT = (17.678 + 2.75) * 25.4; 
double Elevator::GROUND_INTAKE_HEIGHT = LEVELED_HEIGHT + 105;  
double Elevator::PRIMING_HEIGHT = GROUND_INTAKE_HEIGHT + 100;
double Elevator::MAX_HEIGHT = (42 * 25.4);

//double Elevator::ELEVATOR_ERROR_TOLERANCE = 3; 
double Elevator::STACK_HEIGHT = 100;

double Elevator::PRIMING_SPEED = 12;

double Elevator::MINIMUM_ALIGNER_DISTANCE = ROBOT_LENGTH_MM/2 * 1.5;  
double Elevator::ALIGNER_ERROR_TOLERANCE = 10;  

double Elevator::SPOOL_DIAMETER = (Elevator::MAX_HEIGHT - Elevator::LEVELED_HEIGHT) / (2.534 * M_PI);

Elevator& Elevator::getObject(){ 
  return *globalPtr;
}

void Elevator::init(){  

    PIDConstants pidConsts;  
    pidConsts.P = 0.25;
    pidConsts.I = 0.000;
    pidConsts.D = 0.0000;
    pidConsts.errorTolerance = 10;

    lift.setStopping(vex::brakeType::brake); 
    rot.setPosition((GROUND_INTAKE_HEIGHT - LEVELED_HEIGHT) / (M_PI * SPOOL_DIAMETER), vex::rotationUnits::rev);  

    correctionController = new pidcontroller(pidConsts, getPosition());
    correctionController->setLastTimestamp(Brain.Timer.time()); 
    
} 

void Elevator::stop(){ 
   lift.spin(vex::directionType::fwd, 0, vex::voltageUnits::volt);
}

void Elevator::periodic(){ 
   
   double elevatorOutput = 0;
   if (currentState == ElevatorState::E_HOLDING || currentState == ElevatorState::E_PURSUING){//Stay Still
     SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));
     if (pos == SuperStructurePosition::GROUND && get<bool>("at_setpoint")){ 
       elevatorOutput = -4;
     } else { 
       elevatorOutput = correctionController->calculate(getPosition(), Brain.Timer.time());
     }
     //Telemetry::inst.placeValueAt<double>(correctionController->getSetpoint() - getPosition(), "graph", "error"); 
   } else if (currentState == ElevatorState::E_PRIMING){ //Rise or fall at a constant rate
     if (get<bool>("sensing_stack")){
       elevatorOutput = PRIMING_SPEED;
     } else {
       elevatorOutput = 0;
     }
   } else if (currentState == ElevatorState::E_ADJUSTING){ 
     elevatorOutput = PRIMING_SPEED * raisingDirection;
   }
   lift.spin(vex::directionType::rev, elevatorOutput, vex::voltageUnits::volt);
}  

void Elevator::updateTelemetry(){     
    
    set<double>("current_height", getPosition());  
    set<bool>("sensing_stack", primingSensor.objectDistance(vex::distanceUnits::mm) < MINIMUM_ALIGNER_DISTANCE);  
    set<double>("percentage_extended", (get<double>("current_height") - LEVELED_HEIGHT) / (MAX_HEIGHT - LEVELED_HEIGHT));
     
    //Brain.Screen.printAt(20, 120, "Elevator Height: %.2f", getPosition()); 
    stateControl();
    
} 

double Elevator::getPosition(){ 
    return (rot.position(vex::rotationUnits::rev) * M_PI * SPOOL_DIAMETER) + LEVELED_HEIGHT; 
} 

double Elevator::getVelocity(){
   return rot.velocity(vex::velocityUnits::rpm) / 60 * M_PI * SPOOL_DIAMETER; 
}

void Elevator::setSetpoint(double setpoint){  
   if (setpoint == correctionController->getSetpoint()){ 
     return;
   }
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

void Elevator::receiveSetpoints(){ 
  if (get<bool>("requesting_setpoint")){  
      if (get<bool>("sniper_score_enabled")){ 
        primingSetpoint = get<double>("requested_setpoint");
      } else { 
        setSetpoint(get<double>("requested_setpoint"));
      }
      set<bool>("requesting_setpoint", false);
  }
}

bool Elevator::safeToManuever(){  
  SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));
  bool canExitHold; 
  if (pos == SuperStructurePosition::GROUND){ 
      canExitHold = sin(toRadians(Telemetry::inst.getValueAt<double>("forearm", "current_angle"))) < -0.75;
  } else { 
      canExitHold = sin(toRadians(Telemetry::inst.getValueAt<double>("forearm", "current_angle"))) > -0.25; 
  } 
  return canExitHold;
}

void Elevator::maintainHoldLock(){ 
  if (get<bool>("hold")){   
    set<bool>("hold", !safeToManuever());
  }
}

void Elevator::findNextSetpoint(){ 
   SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));
   switch (pos){
      case AUTO:
          if (get<bool>("active")){   
            if (get<bool>("sniper_score_enabled")){ 
              setSetpoint(primingSetpoint); 
              set<bool>("sniper_score_enabled", false);
            } else {  
              currentState = ElevatorState::E_PRIMING; 
            }
          }
          break; 
      case GROUND:  
          set<bool>("requesting_setpoint", true);
          set<double>("requested_setpoint", GROUND_INTAKE_HEIGHT);
          break; 
      case STANDING: 
          set<bool>("requesting_setpoint",true);
          set<double>("requested_setpoint", LEVELED_HEIGHT); 
          break;
      case PRIMED:
          if (get<bool>("sniper_score_enabled")){ 
              setSetpoint(primingSetpoint); 
              set<bool>("sniper_score_enabled", false);
          } else if (!Telemetry::inst.getValueAt<bool>("claw", "in_possession")){ 
              set<bool>("requesting_setpoint", true); 
              set<double>("requested_setpoint", PRIMING_HEIGHT);
          } 
          break;
        default:
          break;
        }
} 

void Elevator::regulatePriming(){ 
  if (!get<bool>("sensing_stack")){ 
      currentState = ElevatorState::E_HOLDING;  
      lock();
      set<bool>("active", false); 
      Telemetry::inst.placeValueAt<bool>(true, "forearm", "active");
  }
}

void Elevator::stateControl(){ 
    receiveSetpoints();
    maintainHoldLock();
    if (currentState == ElevatorState::E_PURSUING){ //
      if (reachedSetpoint()){  
        currentState = ElevatorState::E_HOLDING; 
      }
    } else if (currentState == ElevatorState::E_PRIMING){   
        regulatePriming();
    } else if (!get<bool>("hold")){  
        findNextSetpoint();
    }
    set<bool>("at_setpoint", (currentState == ElevatorState::E_HOLDING || currentState == ElevatorState::E_ADJUSTING) && !get<bool>("hold"));   
    
    raisingDirection = 0;
    if (!RobotState::getStateOf("in_autonomous") && get<bool>("at_setpoint")){ 
       respondToRequests();
    } 
}

void Elevator::respondToRequests(){   
    SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));
    
    if (pos == SuperStructurePosition::PRIMED && Telemetry::inst.getValueAt<bool>("ss_manager", "setpoints_reached") && Telemetry::inst.getValueAt<bool>("claw", "in_possession")){   
      if (RobotState::getStateOf("awaiting_land")){ 
        set<bool>("requesting_setpoint",true);
        set<double>("requested_setpoint", PRIMING_HEIGHT); 
      } else { 
        currentState = E_ADJUSTING; 
        if (RobotState::getStateOf("rise")){ 
          raisingDirection = 1;
        } else if (RobotState::getStateOf("fall")){ 
          raisingDirection = -1; 
        } 
      }
    } 
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

//--------------------------------------------------------------------------------------------- 

void FrontRunElevatorSetpoint::start(){ 
   return;
} 

void FrontRunElevatorSetpoint::periodic(){  
   Telemetry::inst.placeValueAt<bool>(true, "elevator", "sniper_score_enabled");
   Telemetry::inst.placeValueAt<bool>(true, "elevator", "requesting_setpoint"); 
   Telemetry::inst.placeValueAt<double>(elevatorSetpoint, "elevator", "requested_setpoint"); 
   ran = true;
} 

bool FrontRunElevatorSetpoint::isOver(){ 
  return ran; 
} 

void FrontRunElevatorSetpoint::end(){ 
  return;
}