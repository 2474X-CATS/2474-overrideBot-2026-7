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

    PIDConstants pidConsts;  
    pidConsts.P = 0;
    pidConsts.I = 0;
    pidConsts.D = 0;

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

    if (get<bool>("requesting_setpoint")){  
      if (get<bool>("sniper_score_enabled")){ 
        set<double>("priming_setpoint", get<double>("requested_height"));
      } else { 
        setSetpoint(get<double>("requested_height"));
      }
      set<bool>("requesting_setpoint", false); 
    }

    if (currentState == ElevatorState::E_PURSUING){ //
      if (reachedSetpoint()){  
        currentState = ElevatorState::E_HOLDING; 
      }
    } else if (currentState == ElevatorState::E_PRIMING){   
        if (!get<bool>("sensing_stack")){ 
          currentState = ElevatorState::E_HOLDING; 
          set<bool>("active", false); 
          Telemetry::inst.placeValueAt<bool>(true, "forearm", "active");
        }
    } else {  
        switch (pos){
            case AUTO: 
               if (get<bool>("active")){   
                 if (get<bool>("sniper_score_enabled")){ 
                   setSetpoint(get<double>("priming_setpoint"));
                 } else { 
                   currentState = ElevatorState::E_PRIMING; 
                 }
                 set<bool>("sniper_score_enabled", false);
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
            case PRIMED:
               if (get<bool>("sniper_score_enabled")){ 
                  setSetpoint(get<double>("priming_setpoint")); 
                  set<bool>("sniper_score_enabled", false);
               } 
               break;
            default:
               break;
        }
    }   

   set<bool>("at_setpoint", currentState == ElevatorState::E_HOLDING);  
}

void Elevator::respondToRequests(){ 
    SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));
    
    if (pos == SuperStructurePosition::PRIMED && currentState == ElevatorState::E_HOLDING){ 
      
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
    } 
}