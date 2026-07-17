#include "forearm.h" 
#include "../utilities/functools.h"

Forearm& Forearm::getObject(){ 
  return *globalPtr;
}

void Forearm::init(){  

   angularDeadZones[0] = 0.0; 
   angularDeadZones[1] = 0.0;  

   motionConsts.maxAcceleration = 0.0; 
   motionConsts.maxVelocity = 0.0; 
} 

void Forearm::periodic(){ 
    double forearmVelocity; 
    if (currentState == ForearmState::HOLDING){ 
        forearmVelocity = 0; 
    } else { 
        TrapezoidalSetpoint outputGoal = motionProfile->generateSetpoint(Brain.Timer.time());  
        forearmVelocity = outputGoal.velocity * setpointDirection; 
    }  
    updatePosition(forearmVelocity);
} 

void Forearm::updateTelemetry(){ 
    updatePosition(); 
    stateControl();  
    set<bool>("at_setpoint", currentState == ForearmState::HOLDING);
} 

void Forearm::stop(){ 
    return;
}   

void Forearm::updatePosition(double velocity){ 
    double frameTranslation = velocity * ((Brain.Timer.time() - previousTimestamp) / 1000); 
    set<double>("current_angle", angleSum(get<double>("current_angle"), frameTranslation)); 
    previousTimestamp = Brain.Timer.time();
}

bool Forearm::reachedSetpoint(){ 
  return (Brain.Timer.time() - motionProfile->getStartTime()) >= motionProfile->getTotalDuration();
}

void Forearm::setSetpoint(double setpoint, bool inverted){ 
  double currentAngle = getCurrentAngle();
  
  if (inverted){ 
    setpoint = flipOrientation(setpoint); 
  }

  if (setpoint == angularSetpoint){ 
    return; //Already reached or is pursuing
  }

  double error = angleDifference(setpoint, currentAngle);  

  double firstDeadDiff = fabs(angleDifference(angularDeadZones[0], currentAngle));
  double secondDeadDiff = fabs(angleDifference(angularDeadZones[1], currentAngle)); 

  if (fabs(error) > firstDeadDiff && fabs(error) > secondDeadDiff){ 
    error = (360 - fabs(error)) * -1 * copysign(1, error); //Goes through the zone
  } else if (fabs(error) < firstDeadDiff && fabs(error) < secondDeadDiff){ 
    error *= 1;
  } else { 
    return; //You can't go there
  }  

  setpointDirection = copysign(1, error); 

  motionProfile = new TrapezoidalMotionProfile(motionConsts, fabs(error), 0, 0);  
  currentState = ForearmState::PURSUING; 

  angularSetpoint = setpoint;
}

void Forearm::stateControl(){ 
    SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position")); 
    
    if (currentState == ForearmState::PURSUING){ 
        if (reachedSetpoint()){ 
            currentState = ForearmState::HOLDING; 
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
    } else { 
        switch (pos){ 
            case PRIMED:  
              setSetpoint(PRIMING_SETPOINT, RobotState::getStateOf("inverted")); 
              break;
            case GROUND: 
              setSetpoint(GROUND_SETPOINT, RobotState::getStateOf("inverted")); 
              break;
            case STANDING: 
              setSetpoint(STANDING_SETPOINT, RobotState::getStateOf("inverted")); 
              break; 
            case AUTO: 
              if (get<bool>("active")){ 
                if (get<int>("task_id") == 0){ 
                    setSetpoint(PLACE_SETPOINT, RobotState::getStateOf("inverted"));
                } else { 
                    setSetpoint(PRIMING_SETPOINT, RobotState::getStateOf("inverted"));
                }
              } 
              break; 
               
        }
    }
}  

