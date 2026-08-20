#include "forearm.h" 
#include "../utilities/functools.h"

Forearm* Forearm::globalPtr = nullptr; 

double Forearm::PLACE_SETPOINT = 0; 
double Forearm::PRIMING_SETPOINT = 45; 
double Forearm::GROUND_SETPOINT = 270; 
double Forearm::STANDING_SETPOINT = 180;  

double Forearm::ANGULAR_ERROR_TOLERANCE = 3.0;

Forearm& Forearm::getObject(){ 
  return *globalPtr;
}

void Forearm::init(){  
   
   set<double>("current_angle", PRIMING_SETPOINT); 

   angularDeadZones[0] = 135; 
   angularDeadZones[1] = 160;  

   motionConsts.maxAcceleration = 2160; 
   motionConsts.maxVelocity = 1080; 

   pidConsts.P = 0; 
   pidConsts.I = 0; 
   pidConsts.D = 0;  

   feedback = new pidcontroller(pidConsts, 0); 
   
   armFFConsts.kS_rot = 0.0; 
   armFFConsts.kV_rot = 0.0; 
   armFFConsts.kA_rot = 0.0; 
   armFFConsts.kCos = 0.0; 
}

void Forearm::periodic(){ 
    double forearmOutput; 
    if (currentState == ForearmState::F_HOLDING){ 
        forearmOutput = calculateOutput(0,0); 
    } else { 
        TrapezoidalSetpoint outputGoal = motionProfile->generateSetpoint(Brain.Timer.time());  
        forearmOutput = calculateOutput(outputGoal.velocity, outputGoal.acceleration) * setpointDirection; 
    }  
    forearmMotor.spin(vex::directionType::fwd, forearmOutput, vex::voltageUnits::volt);
 
} 

void Forearm::updateTelemetry(){ 
    updatePosition(); 
    stateControl();  
} 

void Forearm::stop(){ 
    forearmMotor.spin(vex::directionType::fwd, calculateOutput(0,0), vex::voltageUnits::volt);
}   

double Forearm::calculateOutput(double omega, double alpha){ 
    double ffOutput = armFFConsts.calculate(getCurrentAngle() / 360, omega / 360, alpha / 360); 
    double pidOutput = feedback->calculate(angleDifference(getVelocity(), omega) , Brain.Timer.time());  
    return ffOutput + pidOutput;
}

double Forearm::getCurrentAngle(){ 
    return rotarySensor.angle(vex::rotationUnits::deg); 
} 

double Forearm::getVelocity(){ 
    return (angleDifference(getCurrentAngle(), previousAngle)) / ((Brain.Timer.time() - previousTimestamp) / 1000.0);
}

void Forearm::updatePosition(){   
    previousAngle = get<double>("current_angle"); 
    set<double>("current_angle", getCurrentAngle());
    previousTimestamp = Brain.Timer.time(); 
}

bool Forearm::reachedSetpoint(){ 
  return (Brain.Timer.time() - motionProfile->getStartTime()) >= motionProfile->getTotalDuration();
}

void Forearm::setSetpoint(double setpoint, bool inverted){ 
  double currentAngle = get<double>("current_angle"); 
  if (inverted){ 
    setpoint = flipOrientation(setpoint); 
  } 
  double error = angleDifference(setpoint, currentAngle);  
  if (fabs(error) < ANGULAR_ERROR_TOLERANCE){  
    return;
  }
  setpointDirection = copysign(1, error); 
  motionProfile = new TrapezoidalMotionProfile(motionConsts, fabs(error), getVelocity(), 0);   
  motionProfile->setLastTimestamp(Brain.Timer.time()); 
  feedback->setLastTimestamp(Brain.Timer.time());
  currentState = ForearmState::F_PURSUING; 
}

void Forearm::stateControl(){ 
    
    SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position")); 

    if (requestingSetpoint){  
      requestingSetpoint = false;
      setSetpoint(requestedSetpoint, RobotState::getStateOf("inverted")); 
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
    } else if (currentState == ForearmState::F_HOLDING) {    
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
                    requestedSetpoint = PRIMING_SETPOINT;
                }
              } 
              break;  
            default: 
              break;
        }  
    }  

    set<bool>("at_setpoint", currentState == ForearmState::F_HOLDING);  
}  

