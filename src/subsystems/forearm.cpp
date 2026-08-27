#include "forearm.h" 
#include "../utilities/functools.h" 

Forearm* Forearm::globalPtr = nullptr; 

double Forearm::PLACE_SETPOINT = 0; 
double Forearm::PRIMING_SETPOINT = 45; 
double Forearm::GROUND_SETPOINT = 0; 
double Forearm::STANDING_SETPOINT = 180;  

//double Forearm::ANGULAR_ERROR_TOLERANCE = 3.0; 

Forearm& Forearm::getObject(){ 
  return *globalPtr;
}

void Forearm::init(){

   forearmMotor.setPosition(0, vex::rotationUnits::deg);  
   forearmMotor.setBrake(vex::brakeType::hold); 

   angularDeadZones[0] = 0.0; 
   angularDeadZones[1] = 0.0;  

   motionConsts.maxAcceleration = 540 / 0.5;
   motionConsts.maxVelocity = 540;

   pidConsts.P = 0.001;
   pidConsts.I = 0.00;
   pidConsts.D = 0;//0.0003;

   feedback = new pidcontroller(pidConsts, 0); 
   
   //v1 = 2
   //v2 = 3.925

   //Tune later (Try modifying kCos)
   armFFConsts.kS_rot = (3.925 - 0.825) / 2; //(3.35 - 0.61) / 2;
   armFFConsts.kV_rot = 4.35;
   armFFConsts.kA_rot = 0.5;//0.625;
   armFFConsts.kCos = 0.825 + (3.925 - 0.825) / 2; //2.5; //0.61 + armFFConsts.kS_rot;  
   //If holding then hold brake type handles (fall)

   startingAngle = 270;
   setpoint = startingAngle; 

   set<double>("requested_angle", 0); 
   set<bool>("requesting_setpoint",true);

   feedback->setLastTimestamp(Brain.Timer.time());

} 

void Forearm::periodic(){ 
    double forearmOutput; 
    
    if (currentState == ForearmState::HOLDING){ 
        forearmOutput = calculateOutput(0,0); 
    } else {
        TrapezoidalSetpoint outputGoal = motionProfile->generateSetpoint(Brain.Timer.time());  
        forearmOutput = calculateOutput(outputGoal.velocity, outputGoal.acceleration);// * setpointDirection; 
    }    

    //Brain.Screen.printAt(20, 120, "Current Voltage: %.2f", forearmOutput); 
    //Brain.Screen.printAt(20, 140, "Current Voltage: %.2f", getCurrentAngle());
    forearmMotor.spin(vex::directionType::fwd, forearmOutput, vex::voltageUnits::volt);
} 

void Forearm::updateTelemetry(){ 
    updatePosition(); 
    respondToRequests();
    stateControl();  
    set<bool>("at_setpoint", currentState == ForearmState::HOLDING); 
    //Brain.Screen.printAt(20, 120, "Current Angle: %.2f", getCurrentAngle()); 
    //Brain.Screen.printAt(20, 140, "Current cos ratio: %.2f", cos(toRadians(getCurrentAngle())));
} 

void Forearm::stop(){
    forearmMotor.spin(vex::directionType::fwd, 0, vex::voltageUnits::volt);
}

double Forearm::calculateOutput(double omega, double alpha){  
    set<double>("requested_velocity", omega);   
    double ffOutput = armFFConsts.calculate(toRadians(getCurrentAngle()), omega / 360, alpha / 360);  
    double pidOutput;
    if (currentState == ForearmState::HOLDING){  
       pidOutput = feedback->calculate(angleDifference(getCurrentAngle(), setpoint), Brain.Timer.time());   
    } else {
       pidOutput = feedback->calculate(angleDifference(getVelocity(), omega), Brain.Timer.time());   
    } 
    
    return ffOutput + pidOutput;
}

double Forearm::getCurrentAngle(){
    return angleSum(angleSum(startingAngle, (forearmMotor.position(vex::rotationUnits::rev) * 180)), 0);
}

double Forearm::getVelocity(){
    return (forearmMotor.velocity(vex::velocityUnits::dps) / 2);
}

void Forearm::updatePosition(){       
    set<double>("current_velocity", getVelocity());   
    set<double>("current_angle", getCurrentAngle());   
}

bool Forearm::reachedSetpoint(){
  return (Brain.Timer.time() - motionProfile->getStartTime()) >= motionProfile->getTotalDuration();
}

void Forearm::setSetpoint(double setpoint, bool inverted){ 
  double currentAngle = getCurrentAngle(); 
  
  double error = angleDifference(setpoint, currentAngle); 
  this->setpoint = setpoint;  

  motionProfile = new TrapezoidalMotionProfile(motionConsts, error, 0, 0);   
  motionProfile->setLastTimestamp(Brain.Timer.time()); 
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
        
        if ((xComponent == 0 && yComponent == 0) && get<double>("last_component_x") + get<double>("last_component_y") == 1){ 
           double angle = atan2(get<double>("last_component_y"), get<double>("last_component_x")); 
           if (angle < 0){
             angle = ((2 * M_PI) + angle);
           } 
           set<double>("requesting_setpoint", true); 
           set<double>("requested_angle", toDegrees(angle));
        } 
        
        set<double>("last_component_x", xComponent);
        set<double>("last_component_y", yComponent); 

        RobotState::manuallyModifyState("scheduling_setpoint", false); 

    }
}