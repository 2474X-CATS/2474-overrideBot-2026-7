#include "elevator.h" 
#include "../utilities/functools.h"

Elevator* Elevator::globalPtr = nullptr;

const double Elevator::LEVELED_HEIGHT = (17.678 + 2.75) * 25.4; 
const double Elevator::GROUND_INTAKE_HEIGHT = LEVELED_HEIGHT + 160;  
const double Elevator::PRIMING_HEIGHT = GROUND_INTAKE_HEIGHT + 150;
const double Elevator::MAX_HEIGHT = (42 * 25.4); 

const double Elevator::GROUND_PRESSURE = -5;

const double Elevator::PRIMING_SPEED = 12;

const double Elevator::MINIMUM_ALIGNER_DISTANCE = ROBOT_LENGTH_MM/2 * 1.5;  
//const double Elevator::ALIGNER_ERROR_TOLERANCE = 10;

const double Elevator::SPOOL_DIAMETER = (Elevator::MAX_HEIGHT - Elevator::LEVELED_HEIGHT) / (2.534 * M_PI);

Elevator& Elevator::getObject(){ 
  return *globalPtr;
}


Elevator::Elevator() : 
    Subsystem( 
        "elevator", 
        { 
            (EntrySet){"active", EntryType::BOOL}, //In a macro?
            (EntrySet){"at_setpoint", EntryType::BOOL}, //Achieved setpoint or no setpoint? 
            (EntrySet){"sensing_stack", EntryType::BOOL}, 
            (EntrySet){"requested_setpoint", EntryType::DOUBLE}, 
            (EntrySet){"requesting_setpoint", EntryType::BOOL},
            (EntrySet){"sniper_score_enabled", EntryType::BOOL},
            (EntrySet){"percentage_extended", EntryType::DOUBLE}, 
            (EntrySet){"current_height", EntryType::DOUBLE},
            (EntrySet){"hold", EntryType::BOOL},  
            (EntrySet){"has_dropped", EntryType::BOOL}
         }
    ),
    lifter1(vex::motor(vex::PORT14, vex::ratio18_1, true)), 
    lifter2(vex::motor(vex::PORT10, vex::ratio18_1)), 
    lift(vex::motor_group(lifter1, lifter2)),
    rot(vex::rotation(vex::PORT2)),
    primingSensor(vex::distance(vex::PORT3))
    { 
        globalPtr = this;
    };


bool Elevator::underGlobalStall(){ 
  return Brain.Timer.time() - Telemetry::inst.getValueAt<double>("ss_manager", "transition_stamp") < Telemetry::inst.getValueAt<double>("ss_manager", "transition_delay");
}

void Elevator::init(){  

    PIDConstants pidConsts;  
    pidConsts.P = 0.25;
    pidConsts.I = 0.000;
    pidConsts.D = 0.0000;
    pidConsts.errorTolerance = 10;

    lift.setStopping(vex::brakeType::hold); 
    rot.setPosition(0, vex::rotationUnits::rev);  

    correctionController = new pidcontroller(pidConsts, getPosition());
    correctionController->setLastTimestamp(Brain.Timer.time()); 
    
} 

void Elevator::stop(){ 
   lift.stop();
}

void Elevator::periodic(){ 
   double elevatorOutput = 0;   
   if (currentState == ElevatorState::E_HOLDING || currentState == ElevatorState::E_PURSUING){//Stay Still
     SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));
     if (pos == SuperStructurePosition::GROUND || (pos == SuperStructurePosition::PRIMED && !Telemetry::inst.getValueAt<bool>("claw", "in_possession")) && get<bool>("at_setpoint")){ 
       elevatorOutput = GROUND_PRESSURE;
     } else if (pos == SuperStructurePosition::STANDING && RobotState::getStateOf("purge")){ 
       elevatorOutput = -12; 
     } else { 
       elevatorOutput = correctionController->calculate(getPosition(), Brain.Timer.time()); 
       //Telemetry::inst.placeValueAt<double>(getPosition(), "graph", "error");  
       //Telemetry::inst.placeValueAt<double>(correctionController->getSetpoint(), "graph", "zero");
     }
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
    set<double>("percentage_extended", (get<double>("current_height") - LEVELED_HEIGHT) / (MAX_HEIGHT - LEVELED_HEIGHT));
    set<bool>("sensing_stack", (primingSensor.objectDistance(vex::distanceUnits::mm) < MINIMUM_ALIGNER_DISTANCE && get<double>("percentage_extended") < 1.35)); 
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
  //bool safe = true; //Telemetry::inst.getValueAt<bool>("forearm", "at_setpoint"); 
  bool safe = true;  

  int pos = Telemetry::inst.getValueAt<int>("ss_manager", "position");
  double forearmAngleVert = sin(toRadians(Telemetry::inst.getValueAt<double>("forearm", "current_angle")));   
  
  if (pos == SuperStructurePosition::GROUND || pos == SuperStructurePosition::PRIMED){ 
    safe = forearmAngleVert < -0.875;
  } 

  return safe;
}

void Elevator::maintainHoldLock(){ 
  if (get<bool>("hold") && !underGlobalStall()){    
    if (Telemetry::inst.getValueAt<bool>("forearm", "hold") && Telemetry::inst.getValueAt<int>("ss_manager", "position") != SuperStructurePosition::AUTO){  
      set<bool>("requesting_setpoint", true); 
      set<double>("requested_setpoint", PRIMING_HEIGHT);
    }
    set<bool>("hold", !safeToManuever());  
  }
}

void Elevator::findNextSetpoint(){ 
   SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));
   switch (pos){
      case AUTO:
          if (get<bool>("active")){   
            currentState = ElevatorState::E_PRIMING; 
          } 
          set<bool>("has_dropped", false);
          break; 
      case GROUND:  
          set<bool>("requesting_setpoint", true);
          set<double>("requested_setpoint", GROUND_INTAKE_HEIGHT); 
          break;
      case STANDING:
          set<bool>("requesting_setpoint",true);  
          if (RobotState::getStateOf("purge")){ 
            set<double>("requested_setpoint", LEVELED_HEIGHT + 50); 
          } else {
            set<double>("requested_setpoint", LEVELED_HEIGHT + 150); 
          }
          break;
      case PRIMED: 
          if (Telemetry::inst.getValueAt<bool>("claw", "in_possession")){ 
            if (get<bool>("sniper_score_enabled")){ 
              setSetpoint(primingSetpoint); 
              set<bool>("sniper_score_enabled", false); 
            } 
            if (!get<bool>("has_dropped")){ 
              set<bool>("requesting_setpoint", true); 
              set<double>("requested_setpoint", LEVELED_HEIGHT + 50); 
            }
            set<bool>("has_dropped", true);
          } else { 
            set<bool>("requesting_setpoint", true); 
            set<double>("requested_setpoint", GROUND_INTAKE_HEIGHT);
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
    
    maintainHoldLock(); 
    receiveSetpoints();
     
    if (currentState == ElevatorState::E_PURSUING){
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
