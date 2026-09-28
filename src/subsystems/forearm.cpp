#include "forearm.h" 
#include "../utilities/functools.h"

Forearm* Forearm::globalPtr = nullptr; 

const double Forearm::PLACE_SETPOINT = 15;
const double Forearm::PRIMING_SETPOINT = 89;
const double Forearm::GROUND_SETPOINT = 276;
const double Forearm::STANDING_SETPOINT = 10;
const double Forearm::RELEASE_SETPOINT = 82.5; 
const double Forearm::SCOOP_SETPOINT = 5.0;//7.5; 
const double Forearm::KCOS = 0.135;

Forearm& Forearm::getObject(){ 
  return *globalPtr;
} 

Forearm::Forearm(): 
    Subsystem( 
        "forearm",
        {  
          (EntrySet){"task_id", EntryType::INT}, 
          (EntrySet){"active", EntryType::BOOL}, 
          (EntrySet){"at_setpoint", EntryType::BOOL}, 
          (EntrySet){"current_angle", EntryType::DOUBLE}, 
          (EntrySet){"hold", EntryType::BOOL} 
        }
    ),
    forearmMotor(vex::motor(vex::PORT4)), 
    rot(vex::rotation(vex::PORT12))
    { 
        globalPtr = this;
    };

void Forearm::init(){  
   forearmMotor.setPosition(0, vex::rotationUnits::deg); 
   forearmMotor.setBrake(vex::brakeType::coast); 

   angularDeadZones[0] = 0;
   angularDeadZones[1] = 0;

   pidConsts.P = 12/90.0;
   pidConsts.I = 0.075;//0.35;//0.0025;//0.02;
   pidConsts.D = 3/360.0;//1.0/540;//0.00625;//0.0075; 
   pidConsts.errorTolerance = 5;

   feedback = new pidcontroller(pidConsts, 0);  

   feedback->setLastTimestamp(Brain.Timer.time());  
  
   startingAngle = 270;
   rot.setPosition(0, vex::rotationUnits::rev); 
   setpoint = startingAngle;
}

void Forearm::periodic(){
    forearmMotor.spin(vex::directionType::fwd, getOutput(), vex::voltageUnits::volt);  
    Brain.Screen.printAt(20, 120, "Forearm Angle: %.2f", cos(toRadians(getCurrentAngle()))); 
    //stop();
}

void Forearm::updateTelemetry(){  
    set<double>("current_angle", getCurrentAngle());
    stateControl();
}

void Forearm::stop(){
    forearmMotor.stop();
}

double Forearm::getOutput(){
    Telemetry::inst.placeValueAt<double>(getError(), "graph", "error"); 
    double pidOutput = feedback->calculate(getError(), Brain.Timer.time()); 
    double standingOutput = (KCOS * cos(toRadians(getCurrentAngle())));
    double output = standingOutput + pidOutput;
    output = max<double>(output, -12);
    output = min<double>(output, 12);
    return output;
}

double Forearm::getError(){
    double angleDiff = angleDifference(getCurrentAngle(), setpoint);
    double currentAngle = toRadians(getCurrentAngle());
    if (cos(currentAngle) < 0){
       if (sin(currentAngle) > 0 && angleDiff < 0){
          angleDiff = 360 + angleDiff;
       } else if (sin(currentAngle) < 0 && angleDiff > 0){
          angleDiff = 360 - angleDiff;
       }
    }
    return angleDiff;
}

double Forearm::getCurrentAngle(){ 
    return angleSum(rot.angle(vex::rotationUnits::deg), startingAngle);
} 

double Forearm::getVelocity(){
    return rot.velocity(vex::velocityUnits::dps);  
}

bool Forearm::reachedSetpoint(){ 
  return feedback->atSetpoint(angleDifference(getCurrentAngle(), setpoint));
}

bool Forearm::safeToManuever(){
  return Telemetry::inst.getValueAt<double>("elevator", "current_height") > 700; 
}
 
void Forearm::maintainHoldLock(){ 
  if (get<bool>("hold")){  
      set<bool>("hold", !safeToManuever());
  }
} 

void Forearm::setSetpoint(double setp, bool inverted){  
  if (setp == setpoint){ 
    return; 
  } 
  setpoint = setp; 
  currentState = ForearmState::F_PURSUING; 
}

void Forearm::receiveSetpoints(){ 
  if (requestingSetpoint){  
    requestingSetpoint = false;
    setSetpoint(requestedSetpoint, RobotState::getStateOf("inverted")); 
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
    switch (pos){  
        case PRIMED:
          requestingSetpoint = true; 
          requestedSetpoint = PRIMING_SETPOINT;
          break;
        case GROUND:  
          requestingSetpoint = true; 
          requestedSetpoint = GROUND_SETPOINT;
          break;
        case STANDING:   
          requestingSetpoint = true;   
          requestedSetpoint = STANDING_SETPOINT;
          break; 
        case AUTO: 
          if (get<bool>("active")){  
            requestingSetpoint = true;
            if (get<int>("task_id") == 0){ 
                requestedSetpoint = PLACE_SETPOINT;
            } else { 
                requestedSetpoint = RELEASE_SETPOINT;
            }
          } 
          break;  
        default: 
          break;
    }  
}

void Forearm::stateControl(){    
  
    receiveSetpoints();  
    maintainHoldLock(); 
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
