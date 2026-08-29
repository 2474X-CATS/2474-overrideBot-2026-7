#include "forearm.h" 
#include "../utilities/functools.h"

Forearm* Forearm::globalPtr = nullptr; 

double Forearm::PLACE_SETPOINT = 0; 
double Forearm::PRIMING_SETPOINT = 45; 
double Forearm::GROUND_SETPOINT = 270; 
double Forearm::STANDING_SETPOINT = 0;   
double Forearm::RELEASE_SETPOINT = 60;

Forearm& Forearm::getObject(){ 
  return *globalPtr;
}

void Forearm::init(){  
   forearmMotor.setPosition(0, vex::rotationUnits::deg); 
   forearmMotor.setBrake(vex::brakeType::coast); 

   //set<double>("current_angle", PRIMING_SETPOINT); 

   angularDeadZones[0] = 0;
   angularDeadZones[1] = 0;

   motionConsts.maxAcceleration = 480; 
   motionConsts.maxVelocity = 480; 

   pidConsts.P = 0.0725; 
   pidConsts.I = 0.075;//0.1;
   pidConsts.D = 0;//0.02;
   pidConsts.errorTolerance = 0.75;

   feedback = new pidcontroller(pidConsts, 0);  

   feedback->setLastTimestamp(Brain.Timer.time());  
  
   armFFConsts.kS_rot = (3.925 - 0.825) / 2;
   armFFConsts.kV_rot = 2.5;
   armFFConsts.kA_rot = 0;
   armFFConsts.kCos = 2;//0.825 + (3.925 - 0.825) / 2;

   startingAngle = 270;
   setpoint = startingAngle;  

   requestingSetpoint = true; 
   requestedSetpoint = 0; 

}

void Forearm::periodic(){
    double forearmOutput;
    if (currentState == ForearmState::F_HOLDING){ 
        forearmOutput = calculateOutput(0,0);
    } else { 
        TrapezoidalSetpoint outputGoal = motionProfile->generateSetpoint(Brain.Timer.time());  
        forearmOutput = calculateOutput(outputGoal.velocity, outputGoal.acceleration); 
    }  
    forearmMotor.spin(vex::directionType::fwd, forearmOutput, vex::voltageUnits::volt);
} 

void Forearm::updateTelemetry(){   
    Brain.Screen.printAt(20, 150, "Current angle: %.2f", getCurrentAngle()); 
    stateControl();  
} 

void Forearm::stop(){ 
    forearmMotor.stop();
}   

double Forearm::calculateOutput(double omega, double alpha){  
    Telemetry::inst.placeValueAt<double>(omega, "graph", "desired_velocity");  
    Telemetry::inst.placeValueAt<double>(getVelocity(), "graph", "actual_velocity"); 
    double ffOutput = armFFConsts.calculate(toRadians(getCurrentAngle()), 0,0/*omega / 360, alpha / 360*/);  
    double pidOutput = feedback->calculate(angleDifference(getCurrentAngle(), setpoint), Brain.Timer.time()); 
    
    double output = ffOutput + pidOutput; 

    pidOutput = max<double>(output, -12);  
    pidOutput = min<double>(output, 12);
  
    /*
    if (currentState == ForearmState::F_PURSUING){ 
        pidOutput = feedback->calculate(angleDifference(getVelocity(), omega), Brain.Timer.time());
    } else { 
        pidOutput = feedback->calculate(angleDifference(getCurrentAngle(), setpoint), Brain.Timer.time());  
    } 
        */
    return pidOutput + ffOutput;
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
  double currentAngle = getCurrentAngle(); 
  double error = angleDifference(setpoint, currentAngle);  
 

  motionProfile = new TrapezoidalMotionProfile(motionConsts, error, 0, 0);   
  motionProfile->setLastTimestamp(Brain.Timer.time()); 
  feedback->setLastTimestamp(Brain.Timer.time());
  currentState = ForearmState::F_PURSUING; 
}

void Forearm::stateControl(){ 
    if (requestingSetpoint){  
      requestingSetpoint = false;
      setSetpoint(requestedSetpoint, RobotState::getStateOf("inverted")); 
    }

    if (currentState == ForearmState::F_PURSUING){  
        if (reachedSetpoint()){ 
          currentState = ForearmState::F_HOLDING;   
          /*
          if (setpoint == 45){ 
            requestedSetpoint = 0; 
            requestingSetpoint = true;
          }  
          */

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
    /*
    if (currentState == ForearmState::F_HOLDING) {     
        SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position")); 
        bool canTransition = Telemetry::inst.getValueAt<bool>("ss_manager", "can_transition"); 
        switch (pos){  
            case PRIMED:
              requestingSetpoint = true; 
              requestedSetpoint = PRIMING_SETPOINT;
              break;
            case GROUND:  
              if (canTransition){ 
                requestingSetpoint = true; 
                requestedSetpoint = GROUND_SETPOINT;
              } 
              break;
            case STANDING:   
              if (canTransition){ 
                requestingSetpoint = true; 
                requestedSetpoint = STANDING_SETPOINT;
              } 
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
    */

    set<bool>("at_setpoint", currentState == ForearmState::F_HOLDING);  
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
