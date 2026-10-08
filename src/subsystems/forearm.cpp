#include "forearm.h" 
#include "../utilities/functools.h"

Forearm* Forearm::globalPtr = nullptr; 

const double Forearm::PLACE_SETPOINT = 10;
const double Forearm::PRIMING_SETPOINT = 90;
const double Forearm::GROUND_SETPOINT = 270;
const double Forearm::STANDING_SETPOINT = 350;

const double Forearm::KCOS = 1.5;

Forearm& Forearm::getObject(){
  return *globalPtr; 
} 

bool Forearm::underGlobalStall(){ 
  return Brain.Timer.time() - Telemetry::inst.getValueAt<double>("ss_manager", "transition_stamp") < Telemetry::inst.getValueAt<double>("ss_manager", "transition_delay");
}

Forearm::Forearm(): 
    Subsystem( 
        "forearm",
        {  
          (EntrySet){"task_id", EntryType::INT}, 
          (EntrySet){"active", EntryType::BOOL}, 
          (EntrySet){"at_setpoint", EntryType::BOOL}, 
          (EntrySet){"current_angle", EntryType::DOUBLE}, 
          (EntrySet){"hold", EntryType::BOOL}, 

          (EntrySet){"requesting_setpoint", EntryType::BOOL}, 
          (EntrySet){"requested_setpoint", EntryType::DOUBLE}
        }
    ),
    forearmMotor(vex::motor(vex::PORT4)), 
    rot(vex::rotation(vex::PORT12))
    { 
        globalPtr = this;
    };

void Forearm::init(){ 
   forearmMotor.setBrake(vex::brakeType::hold);  

   pidConsts.P = 12/75.0;
   pidConsts.I = 0.075;//0.075; 
   pidConsts.D = 0.0125;//3/360.0;
   pidConsts.errorTolerance = 5;

   feedback = new pidcontroller(pidConsts, 0);  
   feedback->setLastTimestamp(Brain.Timer.time());  
   setpoint = 90;
  
}

void Forearm::periodic(){
    forearmMotor.spin(vex::directionType::rev, getOutput(), vex::voltageUnits::volt);    
}

void Forearm::updateTelemetry(){   
    //Brain.Screen.printAt(20, 120, "Current angle: %.2f", getCurrentAngle()); 
    set<double>("current_angle", getCurrentAngle());
    stateControl();
}

void Forearm::stop(){
    forearmMotor.stop();
}

double Forearm::getOutput(){
    Telemetry::inst.placeValueAt<double>(getError(), "graph", "error");  
    Telemetry::inst.placeValueAt<double>(0, "graph", "zero");
    double pidOutput = feedback->calculate(getError(), Brain.Timer.time()); 
    double standingOutput = (KCOS * cos(toRadians(getCurrentAngle())));
    double output = standingOutput + pidOutput;
    output = max<double>(output, -12);
    output = min<double>(output, 12);
    return output;
}

double Forearm::getError(){ 
    double currentAngle = getCurrentAngle();
    double angleDiff = angleDifference(getCurrentAngle(), setpoint);
    double turnDirection = copysign(1, angleDiff); 
    double distFromTaboo = angleDifference(getCurrentAngle(), 180); 
    bool flip = false;  
    if (copysign(1, distFromTaboo) == turnDirection){ 
      if (turnDirection == -1){
        flip = angleDiff < distFromTaboo;
      } else { 
        flip = angleDiff > distFromTaboo;
      }
    } 
    if (flip){ 
      angleDiff = (360 - abs(angleDiff)) * -1 * turnDirection; 
    }
    return angleDiff;
}

double Forearm::getCurrentAngle(){ 
    return angleSum(rot.angle(vex::rotationUnits::deg), 0);
} 

double Forearm::getVelocity(){
    return rot.velocity(vex::velocityUnits::dps);  
}

bool Forearm::reachedSetpoint(){ 
  return feedback->atSetpoint(angleDifference(getCurrentAngle(), setpoint));
}

bool Forearm::safeToManuever(){   
  
  double vertAngComponent = sin(toRadians(getCurrentAngle()));  
  double elevatorHeight = Telemetry::inst.getValueAt<double>("elevator", "current_height"); 
  int pos = Telemetry::inst.getValueAt<int>("ss_manager", "position");  
  
  bool safe = true; 

  if (pos == SuperStructurePosition::STANDING){ 
    safe = elevatorHeight > 700 || vertAngComponent > -0.75;
  } else if (pos == SuperStructurePosition::GROUND && Telemetry::inst.getValueAt<bool>("claw", "in_possession")){ 
    safe = elevatorHeight > 700;
  } else if (pos == SuperStructurePosition::PRIMED){ 
    safe = elevatorHeight > 700;
  }
  
  /*
  if (Telemetry::inst.getValueAt<bool>("claw", "in_possession")){  
    if (vertAngComponent > 0.25 && pos == SuperStructurePosition::GROUND){ 
      safe = Telemetry::inst.getValueAt<double>("elevator", "current_height") > 700; 
    }
  } else { 
    if (vertAngComponent < -0.5 && pos != SuperStructurePosition::GROUND){ 
      safe = Telemetry::inst.getValueAt<double>("elevator", "current_height") > 700; 
    }
  } 
  */
  return safe;
}
 
void Forearm::maintainHoldLock(){ 
  if (get<bool>("hold") && !underGlobalStall()){  
      set<bool>("hold", !safeToManuever());
  }
} 

void Forearm::setSetpoint(double setp){  
  if (setp == setpoint){ 
    return; 
  } 
  setpoint = setp; 
  currentState = ForearmState::F_PURSUING; 
}

void Forearm::receiveSetpoints(){ 
  if (get<bool>("requesting_setpoint")){  
    set<bool>("requesting_setpoint", false); 
    setSetpoint(get<double>("requested_setpoint")); 
  }
} 

void Forearm::passMacroTurn(){ 
   set<bool>("active", false); 
   set<int>("task_id", get<int>("task_id") + 1); 
   if (get<int>("task_id") == 2){ 
        Telemetry::inst.placeValueAt<bool>(true, "ss_manager","task_completed"); 
        set<int>("task_id", 0);  
   } else { 
        Telemetry::inst.placeValueAt(true, "claw","active"); 
   }
}

void Forearm::findNextSetpoint(){ 
    SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));  
    bool inPossession = Telemetry::inst.getValueAt<bool>("claw", "in_possession"); 
    switch (pos){  
        case PRIMED:
          set<bool>("requesting_setpoint", true);
          if (inPossession){ 
               set<double>("requested_setpoint", PRIMING_SETPOINT); 
          } else { 
               set<double>("requested_setpoint", GROUND_SETPOINT);
          }
          break;
        case GROUND:  
          set<bool>("requesting_setpoint", true);
          set<double>("requested_setpoint", GROUND_SETPOINT);
          break;
        case STANDING:
          set<bool>("requesting_setpoint", true);
          set<double>("requested_setpoint", STANDING_SETPOINT);
          break;  
        case AUTO: 
          if (get<bool>("active")){
            set<bool>("requesting_setpoint", true);
            if (get<int>("task_id") == 0){   
                if (setpoint == PLACE_SETPOINT){ 
                  passMacroTurn();
                } else { 
                  set<double>("requested_setpoint", PLACE_SETPOINT);
                }
            } else { 
                set<double>("requested_setpoint", PRIMING_SETPOINT);
            }
          } 
          break;  
        default: 
          break;
    }  
}

void Forearm::stateControl(){     
    maintainHoldLock(); 
    receiveSetpoints();  
    if (currentState == ForearmState::F_PURSUING){  
      if (reachedSetpoint()){ 
          currentState = ForearmState::F_HOLDING;   
          feedback->reset(); 
          if (get<bool>("active")){ 
            passMacroTurn(); 
          } 
      }
    } else if (currentState == ForearmState::F_HOLDING && !get<bool>("hold")) {     
        findNextSetpoint(); 
    }
    
    set<bool>("at_setpoint", currentState == ForearmState::F_HOLDING && !get<bool>("hold"));  
}  


//------------------------------------------------------------------- 

void RunForearm::start(){ 
  return;
} 

void RunForearm::periodic(){ 
  forearmRef.periodic();
} 

bool RunForearm::isOver(){ 
  return false;
} 

void RunForearm::end(){ 
  return;
}
