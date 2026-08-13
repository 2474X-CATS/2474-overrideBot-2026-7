#include "forearm.h" 
#include "../utilities/functools.h"

Forearm* Forearm::globalPtr = nullptr; 

double Forearm::PLACE_SETPOINT = 30; 
double Forearm::PRIMING_SETPOINT = 45; 
double Forearm::GROUND_SETPOINT = 270; 
double Forearm::STANDING_SETPOINT = 0;   
double Forearm::RELEASE_SETPOINT = 75;

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
} 

void Forearm::periodic(){ 
    double forearmVelocity; 
    if (currentState == ForearmState::F_HOLDING){ 
        forearmVelocity = 0; 
    } else { 
        TrapezoidalSetpoint outputGoal = motionProfile->generateSetpoint(Brain.Timer.time());  
        forearmVelocity = outputGoal.velocity * setpointDirection; 
    }  
    updatePosition(forearmVelocity);
} 

void Forearm::updateTelemetry(){  
    stateControl();  
} 

void Forearm::stop(){ 
    return;
}   

void Forearm::updatePosition(double velocity){  
    double frameTranslation = velocity * (20.0/1000); 
    set<double>("current_angle", angleSum(get<double>("current_angle"), frameTranslation)); 
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
  double errorDead1 = angleDifference(angularDeadZones[0], currentAngle);
  double errorDead2 = angleDifference(angularDeadZones[1], currentAngle); 

  if (fabs(error) < ANGULAR_ERROR_TOLERANCE){  
    return;
  }
  
  setpointDirection = copysign(1, error); 

  motionProfile = new TrapezoidalMotionProfile(motionConsts, fabs(error), 0, 0);   
  motionProfile->setLastTimestamp(Brain.Timer.time());
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
              //setSetpoint(PRIMING_SETPOINT, RobotState::getStateOf("inverted")); 
              break;
            case GROUND:  
              requestingSetpoint = true; 
              requestedSetpoint = GROUND_SETPOINT;
              //setSetpoint(GROUND_SETPOINT, RobotState::getStateOf("inverted")); 
              break;
            case STANDING:  
              requestingSetpoint = true; 
              requestedSetpoint = STANDING_SETPOINT;
              //setSetpoint(STANDING_SETPOINT, RobotState::getStateOf("inverted"));  
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

