#include "drivebase.h" 
#include "../utilities/functools.h"

Drivebase* Drivebase::globalPtr = nullptr; 

double Drivebase::MAX_RPM = 450; 
double Drivebase::WHEEL_RADIUS_MM = 2.75 / 2 * 25.4;

void Drivebase::init(){ 
  leftMotors.setStopping(vex::brakeType::brake);  
  rightMotors.setStopping(vex::brakeType::brake);
}  

Drivebase& Drivebase::getObject(){ 
    return *globalPtr;
}

void Drivebase::periodic(){ 
  arcadeDrive(RobotState::getAxisState(AxisType::M_LEFT_VERTICAL), RobotState::getAxisState(AxisType::M_RIGHT_HORIZONTAL));
}

void Drivebase::updateTelemetry(){ 
 return;
}

void Drivebase::stop(){ 
    leftMotors.stop(); 
    rightMotors.stop();
} 

void Drivebase::manualDrive(double voltage){ 
    leftMotors.spin(vex::directionType::rev, voltage, vex::voltageUnits::volt); 
    rightMotors.spin(vex::directionType::fwd, voltage, vex::voltageUnits::volt);
} 

void Drivebase::manualSpin(double voltage){ 
    leftMotors.spin(vex::directionType::rev, -voltage, vex::voltageUnits::volt); 
    rightMotors.spin(vex::directionType::fwd, voltage, vex::voltageUnits::volt); 
} 

void Drivebase::arcadeDrive(double speed, double rotation){ 
    leftMotors.spin(vex::directionType::rev, ((speed + rotation) / 100.0) * 12.0, vex::voltageUnits::volt); 
    rightMotors.spin(vex::directionType::fwd, ((speed - rotation) / 100.0) * 12.0, vex::voltageUnits::volt); 
} 


///-------------------------------------------------------------------------------------- 

double DriveForward::MOTION_CONSTANTS_MAX_VELO = (((Drivebase::MAX_RPM * (2 * Drivebase::WHEEL_RADIUS_MM * M_PI)) / 60.0)); 
double DriveForward::MOTION_CONSTANTS_MAX_ACCEL = DriveForward::MOTION_CONSTANTS_MAX_VELO / (0.25);

double DriveForward::PID_CONSTANTS_KP = 0.001;//75; 
double DriveForward::PID_CONSTANTS_KI = 0;
double DriveForward::PID_CONSTANTS_KD = 0.0000;//01;//005; 

double DriveForward::FF_CONSTANTS_S = 0.71922; 
double DriveForward::FF_CONSTANTS_V = 0.0064; 
double DriveForward::FF_CONSTANTS_A = 0.001275;  


void DriveForward::start(){  
    FFConstants forwardConstants;
    forwardConstants.kS = FF_CONSTANTS_S; 
    forwardConstants.kV = FF_CONSTANTS_V; 
    forwardConstants.kA = FF_CONSTANTS_A;  
    
    PIDConstants pidConstants; 
    pidConstants.P = PID_CONSTANTS_KP; 
    pidConstants.I = PID_CONSTANTS_KI; 
    pidConstants.D = PID_CONSTANTS_KD; 
     
    TrapezoidConstants motionConstants; 
    motionConstants.maxVelocity = MOTION_CONSTANTS_MAX_VELO; 
    motionConstants.maxAcceleration = MOTION_CONSTANTS_MAX_ACCEL; 

    controller = new errorcontroller(pidConstants);  
    motionProfile = new TrapezoidalMotionProfile(motionConstants, distance); 
    ffController = new FeedForward(forwardConstants);  

    controller->setLastTimestamp(Brain.Timer.time()); 
    motionProfile->setLastTimestamp(Brain.Timer.time()); 

} 

void DriveForward::periodic(){   
    TrapezoidalSetpoint motionGoal = motionProfile->generateSetpoint(Brain.Timer.time());   

    double setpointVelocity; 
    double setpointAcceleration; 

    setpointVelocity = motionGoal.velocity; 
    setpointAcceleration = motionGoal.acceleration; 

    //Telemetry::inst.placeValueAt<double>(setpointVelocity, "blueprint", "velocity");

    double ffOutput = ffController->calculate(setpointVelocity, setpointAcceleration) * direction; 
    double correction = controller->calculate(Telemetry::inst.getValueAt<double>("odometry", "velocity_ms"), Brain.Timer.time()); 

    controller->setReference(setpointVelocity * direction);  

    double output = ffOutput + correction;
    
    drivebaseRef.manualDrive(output);  

} 

bool DriveForward::isOver(){ 
    return (Brain.Timer.time() - motionProfile->getStartTime()) >= motionProfile->getTotalDuration();
}

void DriveForward::end(){ 
    drivebaseRef.manualDrive(0);  
    //Telemetry::inst.placeValueAt<double>(0, "blueprint", "velocity");
} 

//------------------------------------------------------------- 


double TurnToHeading::PID_CONSTANTS_KP = 0.0975;
double TurnToHeading::PID_CONSTANTS_KI = 0.002;
double TurnToHeading::PID_CONSTANTS_KD = 0.0035;

void TurnToHeading::start(){ 
   PIDConstants pidConstants;  

   pidConstants.P = PID_CONSTANTS_KP; 
   pidConstants.I = PID_CONSTANTS_KI; 
   pidConstants.D = PID_CONSTANTS_KD;  

   pidConstants.errorTolerance = 1; 
     
   controller = new pidcontroller(pidConstants, 0); 
   controller->setLastTimestamp(Brain.Timer.time()); 

} 

void TurnToHeading::periodic(){ 
    double output = controller->calculate(getError(), Brain.Timer.time());  
    drivebaseRef.manualSpin(output); 
} 

bool TurnToHeading::isOver(){ 
  return controller->atSetpoint(getError());
} 

void TurnToHeading::end(){ 
  drivebaseRef.manualSpin(0);
}
 
double TurnToHeading::getError(){ 
    return angleDifference(setpoint, Telemetry::inst.getValueAt<double>("odometry", "heading_deg")); 
}

