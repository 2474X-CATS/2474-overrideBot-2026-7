#include "forearm.h" 
#include "../utilities/functools.h"

Forearm* Forearm::globalPtr = nullptr; 

double Forearm::PLACE_SETPOINT = 15;
double Forearm::PRIMING_SETPOINT = 75;
double Forearm::GROUND_SETPOINT = 270;
double Forearm::STANDING_SETPOINT = 0;
double Forearm::RELEASE_SETPOINT = 75;

double Forearm::KCOS = 1.45; 

Forearm& Forearm::getObject(){ 
  return *globalPtr;
}

void Forearm::init(){  
   forearmMotor.setPosition(0, vex::rotationUnits::deg); 
   forearmMotor.setBrake(vex::brakeType::coast); 

   angularDeadZones[0] = 0;
   angularDeadZones[1] = 0;

   pidConsts.P = 0.085;
   pidConsts.I = 0.001;//0.0025;//0.02;
   pidConsts.D = 0;//0.00625;//0.0075;
   pidConsts.errorTolerance = 3;

   feedback = new pidcontroller(pidConsts, 0);  

   feedback->setLastTimestamp(Brain.Timer.time());  

   startingAngle = 270;
   setpoint = startingAngle;
}

void Forearm::periodic(){
    forearmMotor.spin(vex::directionType::fwd, getOutput(), vex::voltageUnits::volt);
} 

void Forearm::updateTelemetry(){    
    set<double>("current_angle", getCurrentAngle());
    stateControl();  
} 

void Forearm::stop(){ 
    forearmMotor.stop();
}   

double Forearm::getOutput(){
    double pidOutput = feedback->calculate(angleDifference(getCurrentAngle(), setpoint), Brain.Timer.time()); 
    Telemetry::inst.placeValueAt<double>(angleDifference(getCurrentAngle(), setpoint), "graph", "error");
    double output = (KCOS * cos(toRadians(getCurrentAngle()))) + pidOutput; 

    output = max<double>(output, -10);  
    output = min<double>(output, 10);

    return output;
}

double Forearm::getCurrentAngle(){ 
    return angleSum(startingAngle, (forearmMotor.position(vex::rotationUnits::rev) * 180)); 
} 

double Forearm::getVelocity(){
    return (forearmMotor.velocity(vex::velocityUnits::dps) / 2); 
}

bool Forearm::reachedSetpoint(){ 
  return feedback->atSetpoint(angleDifference(getCurrentAngle(), setpoint)); //(Brain.Timer.time() - motionProfile->getStartTime()) >= motionProfile->getTotalDuration();
}

void Forearm::setSetpoint(double setp, bool inverted){  
  if (setp == setpoint){ 
    return; 
  } 
  setpoint = setp; 
  currentState = ForearmState::F_PURSUING; 
}

void Forearm::stateControl(){ 
    if (requestingSetpoint){  
      requestingSetpoint = false;
      setSetpoint(requestedSetpoint, RobotState::getStateOf("inverted")); 
    }  

    if (get<bool>("hold")){ 
      if (Telemetry::inst.getValueAt<double>("elevator", "current_height") > 9){ 
         set<bool>("hold", false);
      }
    }

    if (currentState == ForearmState::F_PURSUING){  
        if (reachedSetpoint()){ 
          currentState = ForearmState::F_HOLDING;   
          if (get<bool>("active")){ 
            set<bool>("active", false); 
            set<int>("task_id", get<int>("task_id") + 1); 
            if (get<int>("task_id") == 2){ 
              Telemetry::inst.placeValueAt<bool>(true, "ss_manager","task_completed"); 
              set<int>("task_id", 0);  
            } else { 
              Telemetry::inst.placeValueAt(true, "claw","active"); 
            }
          } 
        }
    }    
    
    if (currentState == ForearmState::F_HOLDING) {     
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
