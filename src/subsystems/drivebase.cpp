#include "drivebase.h" 
#include "../utilities/functools.h" 

Drivebase* Drivebase::globalPtr = nullptr; 

const double Drivebase::MAX_RPM = 450; 
const double Drivebase::WHEEL_RADIUS_MM = 2.75 / 2 * 25.4;

const double Drivebase::DRIVE_SENSITIVITY = 1;
const double Drivebase::TURN_SENSITIVITY = 1;

const double Drivebase::TRACK_WIDTH_MM = ROBOT_WIDTH_MM - (2 * 2 * 25.4); 

Drivebase::Drivebase(): 
Subsystem( 
   "drivebase", 
   { 
    (EntrySet){"is_on", EntryType::BOOL}
   }
),
leftFront(vex::motor(vex::PORT18)), //20 true 
leftMid(vex::motor(vex::PORT19)),
leftBack(vex::motor(vex::PORT20)), //19  
rightFront(vex::motor(vex::PORT15)), //17 
rightMid(vex::motor(vex::PORT16)),
rightBack(vex::motor(vex::PORT17)), //18 true   
leftMotors(leftFront, leftBack, leftMid), 
rightMotors(rightFront, rightBack, rightMid)
{   
  globalPtr = this;
}; 

Drivebase::Drivebase(bool usingTendy): //Change ports to fit pushback bot
Subsystem(
   "drivebase",
   {
    (EntrySet){"is_on", EntryType::BOOL}
   }
),
rightFront(vex::motor(vex::PORT4, true)), //20 true 
rightMid(vex::motor(vex::PORT5)),
rightBack(vex::motor(vex::PORT6, true)), //19  

leftFront(vex::motor(vex::PORT1, true)), //17 
leftMid(vex::motor(vex::PORT2)),
leftBack(vex::motor(vex::PORT3, true)), //18 true    

leftMotors(leftFront, leftBack, leftMid), 
rightMotors(rightFront, rightBack, rightMid)
{   
  globalPtr = this;
};

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
    tankDrive(voltageDrive + voltageTurn, voltageDrive - voltageTurn);
}

void Drivebase::tankDrive(double leftSpeed, double rightSpeed){  
  if (RobotState::getStateOf("inverted")){
    leftMotors.spin(vex::directionType::rev, -rightSpeed, vex::voltageUnits::volt);
    rightMotors.spin(vex::directionType::fwd, -leftSpeed, vex::voltageUnits::volt); 
  } else { 
    leftMotors.spin(vex::directionType::rev, leftSpeed, vex::voltageUnits::volt);
    rightMotors.spin(vex::directionType::fwd, rightSpeed, vex::voltageUnits::volt); 
  }
}

void Drivebase::arcadeDrive(double speed, double rotation){   
    speed *= DRIVE_SENSITIVITY / 100 * 12.0; 
    rotation *= TURN_SENSITIVITY / 100 * 12.0;
    
    manualDrive(speed, rotation);
    
}   

///-------------------------------------------------------------------------------------- 


const double DriveForward::PID_CONSTANTS_KP = 12.0/(TILE_SIZE_MM*1.4);//12.0/(TILE_SIZE_MM * (1 - 0.22));//(TILE_SIZE_MM * 1.415); //0.001;//75;
const double DriveForward::PID_CONSTANTS_KI = 0.000;
const double DriveForward::PID_CONSTANTS_KD = 0.000; //0.00198375;//0.00425;

const double DriveForward::STRAIGHTEN_PID_KP = 0.2;//0.05;//0.033
const double DriveForward::STRAIGHTEN_PID_KI = 0;
const double DriveForward::STRAIGHTEN_PID_KD = 0.000;

void DriveForward::start(){
    startX = Telemetry::inst.getValueAt<double>("odometry", "x_position_mm"); 
    startY = Telemetry::inst.getValueAt<double>("odometry", "y_position_mm"); 
    
    initialAngle = Telemetry::inst.getValueAt<double>("odometry", "heading_deg");

    PIDConstants pidConstants;
    pidConstants.P = PID_CONSTANTS_KP;
    pidConstants.I = PID_CONSTANTS_KI;
    pidConstants.D = PID_CONSTANTS_KD;
    pidConstants.minimumOutput = 2.15;
    pidConstants.maximumOutput = 12;
    pidConstants.errorTolerance = 5;
    
    PIDConstants straightenConstants;
    straightenConstants.P = TurnToHeading::PID_CONSTANTS_KP;//STRAIGHTEN_PID_KP; 
    straightenConstants.I = TurnToHeading::PID_CONSTANTS_KI;//STRAIGHTEN_PID_KI; 
    straightenConstants.D = TurnToHeading::PID_CONSTANTS_KD;//STRAIGHTEN_PID_KD;

    controller = new pidcontroller(pidConstants, fabs(distance));   
    straightener = new pidcontroller(straightenConstants, 0); 

    controller->setLastTimestamp(Brain.Timer.time());
    straightener->setLastTimestamp(Brain.Timer.time());

} 

void DriveForward::periodic(){   
    double distTraveled = getDistTraveled();  

    Telemetry::inst.placeValueAt<double>(fabs(distance), "graph", "zero");
    Telemetry::inst.placeValueAt<double>(distTraveled, "graph", "error"); 

    double speed = controller->calculate(distTraveled, Brain.Timer.time()) * copysign(1, distance); 
    double turnCorrection = straightener->calculate(angleDifference(getAnchorAngle(), Telemetry::inst.getValueAt<double>("odometry", "heading_deg")), Brain.Timer.time());
    drivebaseRef.manualDrive(speed, turnCorrection);
}

bool DriveForward::isOver(){
    return controller->atSetpoint(getDistTraveled());
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

const double TurnToHeading::PID_CONSTANTS_KP = 12.0/75;
const double TurnToHeading::PID_CONSTANTS_KI = 0.000;//0.110;
const double TurnToHeading::PID_CONSTANTS_KD = 0.01;

void TurnToHeading::start(){ 
   PIDConstants pidConstants;

   pidConstants.P = PID_CONSTANTS_KP; 
   pidConstants.I = PID_CONSTANTS_KI; 
   pidConstants.D = PID_CONSTANTS_KD; 
   pidConstants.maximumOutput = 8;
   pidConstants.minimumOutput = 2.35; 
   pidConstants.errorTolerance = 0.5;
     
   controller = new pidcontroller(pidConstants, 0); 
   controller->setLastTimestamp(Brain.Timer.time());   

} 

void TurnToHeading::periodic(){
    double error = getError(); 
    //Telemetry::inst.placeValueAt<double>(0, "graph", "zero");
    Telemetry::inst.placeValueAt<double>(error, "graph", "error");
    double output = controller->calculate(error, Brain.Timer.time());   
    drivebaseRef.manualDrive(0, output);
}

bool TurnToHeading::isOver(){ 
  return controller->atSetpoint(getError());
} 

void TurnToHeading::end(){ 
  drivebaseRef.manualDrive(0, 0);  
}
 
double TurnToHeading::getError(){  
    return angleDifference(setpoint, Telemetry::inst.getValueAt<double>("odometry", "heading_deg"));
} 

void TurnToHeading::setAngle(double angle){ 
    setpoint = angle;
} 

//----------------------------------------------------------------------------------------- 

const double Pivot::PIV_PID_CONSTANTS_KP = 12.0 / 75;
const double Pivot::PIV_PID_CONSTANTS_KI = 0;
const double Pivot::PIV_PID_CONSTANTS_KD = 0.01;

void Pivot::start(){ 
   PIDConstants pidConsts;
   pidConsts.P = PIV_PID_CONSTANTS_KP; 
   pidConsts.I = PIV_PID_CONSTANTS_KI; 
   pidConsts.D = PIV_PID_CONSTANTS_KD; 
   pidConsts.errorTolerance = 0.5;
   pidConsts.minimumOutput = 2;
   pidConsts.maximumOutput = 12 / ((arcRadius + Drivebase::TRACK_WIDTH_MM/2) / (Drivebase::TRACK_WIDTH_MM/2));
   //ratio of driving to turning stays the same
   controller = new pidcontroller(pidConsts, 0); 
   controller->setLastTimestamp(Brain.Timer.time()); 
} 

void Pivot::periodic(){  
   
   double currentAngle = Telemetry::inst.getValueAt<double>("odometry", "heading_deg");  
   double error = angleDifference(setpointAngle, currentAngle); 
   
   Telemetry::inst.placeValueAt<double>(error, "graph", "error"); 

   double feedback = controller->calculate(error, Brain.Timer.time()); 

   double majorOutput = feedback * ((arcRadius + Drivebase::TRACK_WIDTH_MM/2.0) / (Drivebase::TRACK_WIDTH_MM/2.0));
   double minorOutput = feedback * ((arcRadius - Drivebase::TRACK_WIDTH_MM/2.0) / (Drivebase::TRACK_WIDTH_MM/2.0)); 

   if (feedback > 0){ 
      drivebaseRef.tankDrive(minorOutput, majorOutput); 
   } else {
      drivebaseRef.tankDrive(majorOutput, minorOutput); 
   } 

}  

bool Pivot::isOver(){   
   double currentAngle = Telemetry::inst.getValueAt<double>("odometry", "heading_deg");
   return controller->atSetpoint(angleDifference(setpointAngle, currentAngle)); 
}

void Pivot::end(){ 
   drivebaseRef.manualDrive(0,0); 
}
//-----------------------------------------------------------------------------------------  

void CurveInto::start(){  

   PIDConstants consts; 
   consts.P = DriveForward::PID_CONSTANTS_KP;
   consts.I = DriveForward::PID_CONSTANTS_KI;
   consts.D = DriveForward::PID_CONSTANTS_KD;
   consts.errorTolerance = 0.5;  
   consts.minimumOutput = 2;
   
   double currentX = Telemetry::inst.getValueAt<double>("odometry", "x_position_mm"); 
   double currentY = Telemetry::inst.getValueAt<double>("odometry", "y_position_mm"); 
   double currentAngle = Telemetry::inst.getValueAt<double>("odometry", "heading_deg");
   double targetTheta = angleSum(toDegrees(atan2(targetY - currentY, targetX - currentX)), 0); 
 
   double thetaTrans = toRadians(angleDifference(targetTheta, currentAngle)); 

   radius = hypot(targetY - currentY, targetX - currentX) / (2 * sin(thetaTrans / 2.0));

   consts.maximumOutput = 12 / ((fabs(radius) + Drivebase::TRACK_WIDTH_MM/2.0) / (Drivebase::TRACK_WIDTH_MM/2.0)); 
   
   double arcLength = fabs(radius) * thetaTrans;  

   controller = new pidcontroller(consts, arcLength); 
   controller->setLastTimestamp(Brain.Timer.time());

} 

void CurveInto::periodic(){ 
   distTraveled += Telemetry::inst.getValueAt<double>("odometry","immediate_distance"); 
   double feedback = controller->calculate(distTraveled, Brain.Timer.time()); 
   double leftOutput = feedback * ((radius - Drivebase::TRACK_WIDTH_MM/2.0) / (Drivebase::TRACK_WIDTH_MM/2.0)); 
   double rightOutput = feedback * ((radius + Drivebase::TRACK_WIDTH_MM/2.0) / (Drivebase::TRACK_WIDTH_MM/2.0)); 
   drivebaseRef.tankDrive(leftOutput, rightOutput);
}
 
bool CurveInto::isOver(){ 
   return controller->atSetpoint(distTraveled);
} 

void CurveInto::end(){ 
   drivebaseRef.tankDrive(0,0);
}

//----------------------------------------------------------------------------------------- 

void DriveToSetpoint::calibrateSetpoints_euc(double currentX, double currentY){ 
   double angle = angleSum(toDegrees(atan2(setpointY - currentY, setpointX - currentX)), 360);
   static_cast<TurnToHeading*>(commands.at(0))->setAngle(angle); 
} 

void DriveToSetpoint::calibrateSetpoints_man_xy(double currentX, double currentY, double currentAngle){ 
  
   double vertAngle = setpointY > currentY ? 90 : 270;
   double horiAngle = setpointX > currentX ? 0 : 180;  
   
   double xDist = fabs(setpointX - currentX); 

   if (fabs(angleDifference(currentAngle, horiAngle)) > 90){ 
     horiAngle = angleSum(horiAngle, 180);
     xDist *= -1;
   } 

   static_cast<TurnToHeading*>(commands.at(0))->setAngle(horiAngle); 
   static_cast<DriveForward*>(commands.at(1))->setDistance(xDist); 
   static_cast<TurnToHeading*>(commands.at(2))->setAngle(vertAngle); 

} 

void DriveToSetpoint::calibrateSetpoints_man_yx(double currentX, double currentY, double currentAngle){ 
   double yDist = fabs(setpointY - currentY);
   
   double vertAngle = setpointY > currentY ? 90 : 270;
   double horiAngle = setpointX > currentX ? 0 : 180;

   if (fabs(angleDifference(currentAngle, vertAngle)) > 90){ 
     vertAngle = angleSum(vertAngle, 180); 
     yDist *= -1;
   } 
   
   static_cast<TurnToHeading*>(commands.at(0))->setAngle(vertAngle); 
   static_cast<DriveForward*>(commands.at(1))->setDistance(yDist);
   static_cast<TurnToHeading*>(commands.at(2))->setAngle(horiAngle); 
   
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
    setAngle(angleBetweenPts(currentX, currentY, targetX, targetY));
    TurnToHeading::start();
};  

//-------------------------------------------------------------------------- 

void ApproachTarget::start(){ 
    double currentX = Telemetry::inst.getValueAt<double>("odometry", "tip_x_position_mm"); 
    double currentY = Telemetry::inst.getValueAt<double>("odometry", "tip_y_position_mm");  
    setDistance(hypot(currentX - targetX, currentY - targetY) - offset);
    DriveForward::start();
} 

//-------------------------------------------------------------------------- 

void AlignWithAxis::start(){ 
   double dist;
   double currentHeading = toRadians(Telemetry::inst.getValueAt<double>("odometry", "heading_deg"));
   if (ax == AlignmentAxis::HORI){ 
      dist = (setpointPos - Telemetry::inst.getValueAt<double>("odometry", "x_position_mm")) / cos(currentHeading);
   } else { 
      dist = (setpointPos - Telemetry::inst.getValueAt<double>("odometry", "y_position_mm")) / sin(currentHeading);
   } 
   setDistance(dist); 
   DriveForward::start(); 
}

//-------------------------------------------------------------------------- 

void MoveToPoint::start(){ 
    PIDConstants turnConsts; 
    PIDConstants driveConsts; 

    turnConsts.P = TurnToHeading::PID_CONSTANTS_KP;
    turnConsts.I = TurnToHeading::PID_CONSTANTS_KI;
    turnConsts.D = TurnToHeading::PID_CONSTANTS_KD; 
    turnConsts.maximumOutput = 12; 
    //turnConsts.minimumOutput = 1.5;

    driveConsts.P = DriveForward::PID_CONSTANTS_KP;
    driveConsts.I = DriveForward::PID_CONSTANTS_KI;
    driveConsts.D = DriveForward::PID_CONSTANTS_KD;  
    driveConsts.maximumOutput = 12; 
    driveConsts.minimumOutput = 1.1;
    driveConsts.errorTolerance = 5;
    
    linController = new pidcontroller(driveConsts, 0); 
    angController = new pidcontroller(turnConsts, 0); 

    linController->setLastTimestamp(Brain.Timer.time());
    angController->setLastTimestamp(Brain.Timer.time());  
    
    double currentX = Telemetry::inst.getValueAt<double>("odometry", "x_position_mm"); 
    double currentY = Telemetry::inst.getValueAt<double>("odometry", "y_position_mm"); 

    double theta = angleSum(toDegrees(atan2(setpY - currentY, setpX - currentX)), 180);  
    
    setpX += cos(toRadians(theta)) * offsetDist; 
    setpY += sin(toRadians(theta)) * offsetDist; 
    
}  

void MoveToPoint::periodic(){ 
    double distError = getLinError();
    double thetaError = getAngError();

    double angOutput = angController->calculate(thetaError, Brain.Timer.time());
    double linOutput = -linController->calculate(distError, Brain.Timer.time());
  
    double discount = 1 - std::min<double>(fabs(thetaError) / 30, 1); 

    linOutput *= discount;

    drivebaseRef.manualDrive(linOutput, angOutput);
} 

bool MoveToPoint::isOver(){ 
    return linController->atSetpoint(getLinError()); 
} 

void MoveToPoint::end(){ 
    drivebaseRef.manualDrive(0,0); 
} 

double MoveToPoint::getLinError(){    
    double currentX, currentY;
    if (cent){ 
       currentX = Telemetry::inst.getValueAt<double>("odometry", "x_position_mm"); 
       currentY = Telemetry::inst.getValueAt<double>("odometry", "y_position_mm");
    } else { 
       currentX = Telemetry::inst.getValueAt<double>("odometry", "tip_x_position_mm"); 
       currentY = Telemetry::inst.getValueAt<double>("odometry", "tip_y_position_mm");
    }
    return hypot(setpX - currentX, setpY - currentY); 
}

double MoveToPoint::getAngError(){  
    double currentX = Telemetry::inst.getValueAt<double>("odometry", "x_position_mm"); 
    double currentY = Telemetry::inst.getValueAt<double>("odometry", "y_position_mm"); 

    double refHeading = angleSum(toDegrees(atan2(setpY - currentY, setpX - currentX)), 0);  

    return angleDifference(refHeading, Telemetry::inst.getValueAt<double>("odometry", "heading_deg"));
}

//--------------------------------------------------------------------------

/*
void Toggle::start(){ 
    startTime = Brain.Timer.time(); 
} 

void Toggle::periodic(){ 
   int timePassed = Brain.Timer.time() - startTime; 
   if (timePassed < 400){  
      drivebaseRef.manualDrive(4, 0); 
   } else { 
      drivebaseRef.manualDrive(-7, 0);
   }
}  

bool Toggle::isOver(){ 
  return Brain.Timer.time() - startTime >= 750;
} 

void Toggle::end(){ 
  drivebaseRef.manualDrive(0,0);
}
*/
