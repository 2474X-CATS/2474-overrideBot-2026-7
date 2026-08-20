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

   angularDeadZones[0] = 0.0; 
   angularDeadZones[1] = 0.0;  

   motionConsts.maxAcceleration = 0.0; 
   motionConsts.maxVelocity = 0.0; 

   pidConsts.P = 0; 
   pidConsts.I = 0; 
   pidConsts.D = 0;  

   feedback = new pidcontroller(pidConsts); 
   
   armFFConsts.kS_rot = 0.0; 
   armFFConsts.kV_rot = 0.0; 
   armFFConsts.kA_rot = 0.0; 
   armFFConsts.kCos = 0.0; 

} 

void Forearm::periodic(){ 
    double forearmOutput; 
    if (currentState == ForearmState::HOLDING){ 
        forearmOutput = calculateOutput(0,0); 
    } else { 
        TrapezoidalSetpoint outputGoal = motionProfile->generateSetpoint(Brain.Timer.time());  
        forearmOutput = calculateOutput(outputGoal.velocity, outputGoal.acceleration) * setpointDirection; 
    }  
    forearmMotor.spin(vex::directionType::fwd, forearmOutput, vex::voltageUnits::volt);
} 

void Forearm::updateTelemetry(){ 
    updatePosition(); 
    respondToRequests();
    stateControl();  
    set<bool>("at_setpoint", currentState == ForearmState::HOLDING);
} 

void Forearm::stop(){ 
    forearmMotor.spin(vex::directionType::fwd, calculateOutput(0,0), vex::voltageUnits::volt);
}   

double Forearm::calculateOutput(double omega, double alpha){  
    set<double>("requested_velocity", omega); 
    double ffOutput = armFFConsts.calculate(getCurrentAngle() / 360, omega / 360, alpha / 360); 
    double pidOutput = feedback->calculate(angleDifference(getVelocity(), omega), Brain.Timer.time());  
    return ffOutput + pidOutput;
}

double Forearm::getCurrentAngle(){ 
    return rotarySensor.angle(vex::rotationUnits::deg); 
} 

double Forearm::getVelocity(){ 
    return (angleDifference(getCurrentAngle(), previousAngle)) / ((Brain.Timer.time() - previousTimestamp) / 1000.0);
}

void Forearm::updatePosition(){       
    set<double>("current_velocity", getVelocity());
    previousAngle = get<double>("current_angle");    
    set<double>("current_angle", getCurrentAngle());  
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
  double error = angleDifference(setpoint, currentAngle); 

  if (fabs(error) < ANGULAR_ERROR_TOLERANCE){  
    return;
  } 

  set<double>("setpoint", setpoint);
  setpointDirection = copysign(1, error); 

  motionProfile = new TrapezoidalMotionProfile(motionConsts, fabs(error), getVelocity(), 0);   
  motionProfile->setLastTimestamp(Brain.Timer.time()); 
  feedback->setLastTimestamp(Brain.Timer.time());
  currentState = ForearmState::PURSUING;  
 
}

void Forearm::stateControl(){  

    if (get<bool>("requesting_setpoint")){  
      setSetpoint(get<double>("requested_angle"), RobotState::getStateOf("inverted"));   
      set<bool>("requesting_setpoint", false);
    } 

    if (currentState == ForearmState::PURSUING){ 
        if (reachedSetpoint()){ 
            currentState = ForearmState::HOLDING; 
        }
    }
}  

void Forearm::respondToRequests(){ 
    if (RobotState::getStateOf("scheduling_setpoint")){  

        int xComponent = RobotState::getAxisState(AxisType::M_LEFT_HORIZONTAL); 
        int yComponent = RobotState::getAxisState(AxisType::M_LEFT_VERTICAL);
        
        if (xComponent == 0 && yComponent == 0){ 
           double angle = atan2(yComponent, xComponent); 
           if (angle < 0){ 
             angle = ((2 * M_PI) + angle);
           } 
           set<double>("requesting_setpoint", true); 
           set<double>("requested_angle", toDegrees(angle));
        } 

        RobotState::manuallyModifyState("scheduling_setpoint", false); 

    }
}