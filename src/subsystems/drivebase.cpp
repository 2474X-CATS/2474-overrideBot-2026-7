#include "drivebase.h" 
#include "../utilities/functools.h"

Drivebase* Drivebase::globalPtr = nullptr; 

double Drivebase::MAX_RPM = 450; 
double Drivebase::WHEEL_RADIUS_MM = 2.75 / 2 * 25.4;

double Drivebase::DRIVE_SENSITIVITY = 1; 
double Drivebase::TURN_SENSITIVITY = 1;

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

void Drivebase::manualDrive(double voltageDrive, double voltageTurn){ 
    leftMotors.spin(vex::directionType::rev, voltageDrive + voltageTurn, vex::voltageUnits::volt); 
    rightMotors.spin(vex::directionType::fwd, voltageDrive - voltageTurn, vex::voltageUnits::volt);
} 


void Drivebase::arcadeDrive(double speed, double rotation){  
    speed *= DRIVE_SENSITIVITY; 
    rotation *= TURN_SENSITIVITY;
    leftMotors.spin(vex::directionType::rev, (((speed + rotation) / 100.0) * 12.0), vex::voltageUnits::volt); 
    rightMotors.spin(vex::directionType::fwd, ((speed - rotation) / 100.0) * 12.0, vex::voltageUnits::volt); 
} 


///-------------------------------------------------------------------------------------- 


double DriveForward::MOTION_CONSTANTS_MAX_VELO = (((Drivebase::MAX_RPM * (2 * Drivebase::WHEEL_RADIUS_MM * M_PI)) / 60.0)) * 0.85; //0.75; 
double DriveForward::MOTION_CONSTANTS_MAX_ACCEL = DriveForward::MOTION_CONSTANTS_MAX_VELO / 0.5;

double DriveForward::PID_CONSTANTS_KP = 0.00075;//0.001;//75;
double DriveForward::PID_CONSTANTS_KI = 0;
double DriveForward::PID_CONSTANTS_KD = 0.000;

double DriveForward::FF_CONSTANTS_S = 0.71922;
double DriveForward::FF_CONSTANTS_V = 0.00583825;
double DriveForward::FF_CONSTANTS_A = 0.0009; 

double DriveForward::STRAIGHTEN_PID_KP = 0.115;//0.05;//0.033
double DriveForward::STRAIGHTEN_PID_KI = 0;
double DriveForward::STRAIGHTEN_PID_KD = 0.000;


void DriveForward::start(){ 

    startX = Telemetry::inst.getValueAt<double>("odometry", "x_position_mm"); 
    startY = Telemetry::inst.getValueAt<double>("odometry", "y_position_mm"); 
    
    initialAngle = Telemetry::inst.getValueAt<double>("odometry", "heading_deg");

    direction = copysign(1, distance); 
    distance = fabs(distance);  

    Telemetry::inst.placeValueAt<double>(distance, "blueprint", "desired_position");
     
    FFConstants forwardConstants;
    forwardConstants.kS = FF_CONSTANTS_S; 
    forwardConstants.kV = FF_CONSTANTS_V; 
    forwardConstants.kA = FF_CONSTANTS_A;  
    
    PIDConstants pidConstants; 
    pidConstants.P = PID_CONSTANTS_KP; 
    pidConstants.I = PID_CONSTANTS_KI; 
    pidConstants.D = PID_CONSTANTS_KD;  
    
    PIDConstants straightenConstants; 
    straightenConstants.P = STRAIGHTEN_PID_KP; 
    straightenConstants.I = STRAIGHTEN_PID_KI; 
    straightenConstants.D = STRAIGHTEN_PID_KD;
     
    TrapezoidConstants motionConstants; 
    motionConstants.maxVelocity = MOTION_CONSTANTS_MAX_VELO; 
    motionConstants.maxAcceleration = MOTION_CONSTANTS_MAX_ACCEL; 

    controller = new pidcontroller(pidConstants, 0);   
    straightener = new pidcontroller(straightenConstants, 0); 
    ffController = new FeedForward(forwardConstants);   

    motionProfile = new TrapezoidalMotionProfile(motionConstants, distance); 
    
    controller->setLastTimestamp(Brain.Timer.time()); 
    motionProfile->setLastTimestamp(Brain.Timer.time());    
    straightener->setLastTimestamp(Brain.Timer.time());
    
    /*  
    PIDConstants consts;  

    consts.P = 0.017; 
    consts.I = 0.001; 
    consts.D = 0; 
    consts.errorTolerance = 10;  

    pidControl = new pidcontroller(consts, distance); 
    pidControl->setLastTimestamp(Brain.Timer.time());   
    */

} 

void DriveForward::periodic(){    
    TrapezoidalSetpoint motionGoal = motionProfile->generateSetpoint(Brain.Timer.time());   

    double setpointVelocity = motionGoal.velocity; 
    double setpointAcceleration = motionGoal.acceleration; 

    Telemetry::inst.placeValueAt<double>(setpointVelocity, "blueprint", "desired_velocity"); 
    Telemetry::inst.placeValueAt<double>(getDistTraveled(), "blueprint", "progress");

    double ffOutput = ffController->calculate(setpointVelocity, setpointAcceleration);  
    double correction = controller->calculate(Telemetry::inst.getValueAt<double>("odometry", "velocity_ms") - setpointVelocity, Brain.Timer.time()); 
    
    double turnCorrection = straightener->calculate(angleDifference(initialAngle, Telemetry::inst.getValueAt<double>("odometry", "heading_deg")), Brain.Timer.time());

    double output = ffOutput + correction;
    
    drivebaseRef.manualDrive(output, turnCorrection);

    /*
    double progress = getDistTraveled();
    Telemetry::inst.placeValueAt<double>(getDistTraveled(), "blueprint", "progress");
    double output = pidControl->calculate(progress, Brain.Timer.time()); 
    output = max<double>(output, -12); 
    output = min<double>(output, 12); 
    output *= direction; 
    drivebaseRef.manualDrive(output);   
    */

} 

bool DriveForward::isOver(){ 
    return (Brain.Timer.time() - motionProfile->getStartTime()) >= motionProfile->getTotalDuration();
}

double DriveForward::getDistTraveled(){ 
    return hypot( 
        startX - Telemetry::inst.getValueAt<double>("odometry", "x_position_mm"),  
        startY - Telemetry::inst.getValueAt<double>("odometry", "y_position_mm")
    );
}

void DriveForward::end(){ 
    drivebaseRef.manualDrive(0, 0);
} 

void DriveForward::setDistance(double dist){ 
    distance = dist;
}

//------------------------------------------------------------- 

double TurnToHeading::PID_CONSTANTS_KP = 0.105;
double TurnToHeading::PID_CONSTANTS_KI = 0.00175;//0.110;
double TurnToHeading::PID_CONSTANTS_KD = 0.0021;

void TurnToHeading::start(){ 
   PIDConstants pidConstants;

   pidConstants.P = PID_CONSTANTS_KP; 
   pidConstants.I = PID_CONSTANTS_KI; 
   pidConstants.D = PID_CONSTANTS_KD;  

   pidConstants.errorTolerance = 1; 
     
   controller = new pidcontroller(pidConstants, 0); 
   controller->setLastTimestamp(Brain.Timer.time());   

   Telemetry::inst.placeValueAt<double>(0, "blueprint", "desired_position"); 


} 

void TurnToHeading::periodic(){   
    double error = getError();
    Telemetry::inst.placeValueAt<double>(error, "blueprint", "progress");
    double output = controller->calculate(error, Brain.Timer.time());   
    output = max<double>(output, -8);
    output = min<double>(output, 8);
    drivebaseRef.manualDrive(0, output); 
} 

bool TurnToHeading::isOver(){ 
  return controller->atSetpoint(getError());
} 

void TurnToHeading::end(){ 
  drivebaseRef.manualDrive(0, 0); 
  vex::this_thread::sleep_for(1000);
}
 
double TurnToHeading::getError(){ 
    return angleDifference(setpoint, Telemetry::inst.getValueAt<double>("odometry", "heading_deg")); 
} 

void TurnToHeading::setAngle(double angle){ 
    setpoint = angle;
}

//----------------------------------------------------------------------------------------- 

void DriveToSetpoint::calibrateSetpoints_euc(double currentX, double currentY){ 
   double angle = angleSum(toDegrees(atan2(setpointY - currentY, setpointX - currentX)), 360);
   double distance = hypot(setpointX - currentX, setpointY - currentY); 
   
   static_cast<TurnToHeading*>(commands.at(0))->setAngle(angle); 
   static_cast<DriveForward*>(commands.at(1))->setDistance(distance);
} 

void DriveToSetpoint::calibrateSetpoints_man_xy(double currentX, double currentY, double currentAngle){ 
   double xDist = setpointX - currentX; 
   double yDist = setpointY - currentY;  
   
   double vertAngle = yDist > 0 ? 90 : 270;
   double horiAngle = xDist > 0 ? 0 : 180;  

   if (fabs(angleDifference(currentAngle, horiAngle)) > 90){ 
     horiAngle = angleSum(horiAngle, 180); 
     xDist *= -1;
   } 

   static_cast<TurnToHeading*>(commands.at(0))->setAngle(horiAngle); 
   static_cast<DriveForward*>(commands.at(1))->setDistance(xDist); 
   static_cast<TurnToHeading*>(commands.at(2))->setAngle(vertAngle); 
   static_cast<DriveForward*>(commands.at(3))->setDistance(yDist);

} 

void DriveToSetpoint::calibrateSetpoints_man_yx(double currentX, double currentY, double currentAngle){ 
   double xDist = setpointX - currentX; 
   double yDist = setpointY - currentY;  
   
   double vertAngle = yDist > 0 ? 90 : 270;
   double horiAngle = xDist > 0 ? 0 : 180;  

   if (fabs(angleDifference(currentAngle, vertAngle)) > 90){ 
     vertAngle = angleSum(vertAngle, 180); 
     yDist *= -1;
   } 
   
   static_cast<TurnToHeading*>(commands.at(0))->setAngle(vertAngle); 
   static_cast<DriveForward*>(commands.at(1))->setDistance(yDist);
   static_cast<TurnToHeading*>(commands.at(2))->setAngle(horiAngle); 
   static_cast<DriveForward*>(commands.at(3))->setDistance(xDist); 
   
} 

void DriveToSetpoint::start(){  
    double currentX = Telemetry::inst.getValueAt<double>("odometry", "x_position_mm"); 
    double currentY = Telemetry::inst.getValueAt<double>("odometry", "y_position_mm"); 
    double currentAngle = Telemetry::inst.getValueAt<double>("odometry", "heading_deg"); 

    switch (path){ 
        case EUCLIDEAN:  
          calibrateSetpoints_euc(currentX, currentY); 
          break;
        case MANHATTAN_XY:  
          calibrateSetpoints_man_xy(currentX, currentY, currentAngle); 
          break;
        case MANHATTAN_YX:  
          calibrateSetpoints_man_yx(currentX, currentY, currentAngle);  
          break;
    }  

    SequentialCommandGroup::start();
} 

//------------------------------------------------------------------------ 

void FaceTarget::start(){ 
    double currentX = Telemetry::inst.getValueAt<double>("odometry", "x_position_mm"); 
    double currentY = Telemetry::inst.getValueAt<double>("odometry", "y_position_mm");  
    setAngle(angleBetweenPts(targetX, targetY, currentX, currentY));
    TurnToHeading::start();
}; 

//-------------------------------------------------------------------------- 

void ApproachTarget::start(){ 
    double currentX = Telemetry::inst.getValueAt<double>("odometry", "x_position_mm"); 
    double currentY = Telemetry::inst.getValueAt<double>("odometry", "y_position_mm");  
    setDistance(hypot(currentX - targetX, currentY - targetY));
    DriveForward::start();
}