#include "elevator.h" 
#include "../utilities/functools.h"

Elevator* Elevator::globalPtr = nullptr;

const double Elevator::MAX_HEIGHT = (47 * 25.4); 
const double Elevator::PRIMING_SPEED = 12;  
const int Elevator::MACRO_DURATION = 250;

const double Elevator::SPOOL_DIAMETER = Elevator::MAX_HEIGHT / (3.74 * M_PI);

Elevator& Elevator::getObject(){
  return *globalPtr;
}

Elevator::Elevator() : 
    Subsystem( 
        "elevator", 
        { 
            (EntrySet){"active", EntryType::BOOL}, //In a macro?
            (EntrySet){"requested_setpoint", EntryType::DOUBLE},
            (EntrySet){"requesting_setpoint", EntryType::BOOL},
            (EntrySet){"percentage_extended", EntryType::DOUBLE},
        }
    ),
    lifter1(vex::motor(vex::PORT2, vex::ratio18_1, true)), 
    lifter2(vex::motor(vex::PORT1, vex::ratio18_1)),
    lifter3(vex::motor(vex::PORT9, vex::ratio18_1, true)),
    lift(vex::motor_group(lifter1, lifter2, lifter3)),
    rot(vex::rotation(vex::PORT3))
    {
        globalPtr = this;
    };

void Elevator::init(){  
    
    lift.setStopping(vex::brakeType::hold); 

    rot.setReversed(true);
    rot.setPosition(0, vex::rotationUnits::rev); 

    currentSetpoint = -5;

} 

void Elevator::stop(){ 
   lift.stop();
}

void Elevator::periodic(){
   double elevatorOutput;
   if (currentState == ElevatorState::E_PURSUING){//Stay Still    
     if (currentSetpoint > getPosition()){ 
       elevatorOutput = 7;
     } else { 
       elevatorOutput = -7;
     }
   } else if (currentState == ElevatorState::E_HOLDING){ 
     if (raisingDirection == 0){ 
       elevatorOutput = (get<double>("percentage_extended") - 0.5) * 1.35;
     } else {
       elevatorOutput = PRIMING_SPEED * raisingDirection;
     }
   } else {
     elevatorOutput = -6;
   } 
   lift.spin(vex::directionType::fwd, elevatorOutput, vex::voltageUnits::volt);
}

void Elevator::updateDrivercontrol(){  
  currentState = E_HOLDING;
  if (RobotState::getStateOf("rise")){ 
     raisingDirection = 1;
  } else if (RobotState::getStateOf("fall")){ 
     raisingDirection = -1;
  } else { 
     raisingDirection = 0;
  }   
} 

void Elevator::updateAuto(){ 
    if (currentState == ElevatorState::E_PURSUING){
      if (reachedSetpoint()){
        currentState = ElevatorState::E_HOLDING; 
      }
    } else if (currentState == ElevatorState::E_SCORING){ 
        regulatePriming();
    } else {
        findNextSetpoint();
    }  
    
    //Testing purposes-- get rid of when making autos

}

void Elevator::updateTelemetry(){ 
    set<double>("percentage_extended", getPosition() / MAX_HEIGHT); 
    if (RobotState::getStateOf("in_autonomous")){ 
      updateAuto();
    } else { 
      updateDrivercontrol();
    }
} 

double Elevator::getPosition(){ 
    return (rot.position(vex::rotationUnits::rev) * M_PI * SPOOL_DIAMETER); 
} 

void Elevator::setSetpoint(double setpoint){   
   if (currentSetpoint == setpoint){ 
      return;
   }
   currentSetpoint = setpoint;
   currentState = ElevatorState::E_PURSUING;
}

bool Elevator::reachedSetpoint(){
   return fabs(currentSetpoint - getPosition()) < 10;
}

void Elevator::findNextSetpoint(){ 
   SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));
   switch (pos){
      case MATCHLOADING:
          setSetpoint(-5);
          break;
      case AUTO:
          currentSetpoint = -1;
          if (get<bool>("active")){
            currentState = ElevatorState::E_SCORING;
            scoreStamp = Brain.Timer.time(); 
          }
          break;
      case PRIMED:
          if (get<bool>("requesting_setpoint")){
            setSetpoint(get<double>("requested_setpoint"));
            set<bool>("requesting_setpoint", false);
          } 
          break;
      default: 
          break;
    }
} 

void Elevator::regulatePriming(){
  if (Brain.Timer.time() - scoreStamp >= MACRO_DURATION){
      currentState = ElevatorState::E_HOLDING;
      set<bool>("active", false);
      Telemetry::inst.placeValueAt<bool>(true, "claw", "active");
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

const double FrontRunElevatorSetpoint::STACK_HEIGHT = 4 * 25.4; 

void FrontRunElevatorSetpoint::start(){ 
  return;
} 

void FrontRunElevatorSetpoint::periodic(){ 
  Telemetry::inst.placeValueAt<bool>(true, "elevator", "requesting_setpoint"); 
  Telemetry::inst.placeValueAt<double>(stacks * STACK_HEIGHT + offset, "elevator", "requested_setpoint");  
  ran = true;
} 

bool FrontRunElevatorSetpoint::isOver(){ 
  return ran;
} 

void FrontRunElevatorSetpoint::end(){ 
  return;
} 