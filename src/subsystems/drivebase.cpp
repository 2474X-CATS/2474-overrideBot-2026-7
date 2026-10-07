#include "drivebase.h" 
#include "../utilities/functools.h" 

Drivebase* Drivebase::globalPtr = nullptr; 

const double Drivebase::MAX_RPM = 450; 
const double Drivebase::WHEEL_RADIUS_MM = 2.75 / 2 * 25.4;

const double Drivebase::DRIVE_SENSITIVITY = 1; 
const double Drivebase::TURN_SENSITIVITY = 1; 

const double Drivebase::ACCELERATION_LIMIT_LIN = 12 / 0.00001;
const double Drivebase::ACCELERATION_LIMIT_ANG = 12 / 0.00001;

const double Drivebase::MAX_LIN_SPEED = 12;
const double Drivebase::MAX_ANG_SPEED = 12;

Drivebase::Drivebase(): 
Subsystem( 
   "drivebase", 
   { 
    (EntrySet){"is_on", EntryType::BOOL}
   }
),
leftFront(vex::motor(vex::PORT20, true)), //20 true
leftBack(vex::motor(vex::PORT19)), //19 
rightFront(vex::motor(vex::PORT17)), //17
rightBack(vex::motor(vex::PORT18, true)), //18 true   
leftMotors(leftFront, leftBack), 
rightMotors(rightFront, rightBack)
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
    if (RobotState::getStateOf("inverted")){ 
      voltageDrive *= -1;
    }
    leftMotors.spin(vex::directionType::fwd, voltageDrive + voltageTurn, vex::voltageUnits::volt); 
    rightMotors.spin(vex::directionType::fwd, voltageDrive - voltageTurn, vex::voltageUnits::volt); 
} 

void Drivebase::arcadeDrive(double speed, double rotation){   
    speed *= DRIVE_SENSITIVITY / 100 * 12.0; 
    rotation *= TURN_SENSITIVITY / 100 * 12.0;
    
    speed = std::min<double>(std::min<double>(speed, lastLinearVoltage + ((20/1000.0) * ACCELERATION_LIMIT_LIN)), MAX_LIN_SPEED);
    speed = std::max<double>(std::max<double>(speed, lastLinearVoltage - ((20/1000.0) * ACCELERATION_LIMIT_LIN)), -MAX_LIN_SPEED); 
    
    rotation = std::min<double>(std::min<double>(rotation, lastAngularVoltage + ((20/1000.0) * ACCELERATION_LIMIT_ANG)), MAX_ANG_SPEED);
    rotation = std::max<double>(std::max<double>(rotation, lastAngularVoltage - ((20/1000.0) * ACCELERATION_LIMIT_ANG)), -MAX_ANG_SPEED); 
     
    manualDrive(speed, rotation);

    lastAngularVoltage = rotation; 
    lastLinearVoltage = speed;
}   


void Drivebase::setSpeeds(double linearVelocity, double angularVelocity){ 
    double linRPM = linearVelocity / (WHEEL_RADIUS_MM * 2 * M_PI) * 60 * 600 / MAX_RPM; 
    double angRPM = ((ROBOT_WIDTH_MM * M_PI) * (angularVelocity / 360.0)) / (WHEEL_RADIUS_MM * 2 * M_PI) * 60 * 600 / MAX_RPM; 
    leftMotors.setVelocity(linRPM - angRPM, vex::velocityUnits::rpm);
    rightMotors.setVelocity(linRPM + angRPM, vex::velocityUnits::rpm); 
    leftMotors.spin(vex::directionType::fwd); 
    rightMotors.spin(vex::directionType::fwd);
}


///-------------------------------------------------------------------------------------- 

const double DriveForward::MOTION_CONSTANTS_MAX_VELO = (((Drivebase::MAX_RPM * (2 * Drivebase::WHEEL_RADIUS_MM * M_PI)) / 60.0)) * 0.85; //0.75; 
const double DriveForward::MOTION_CONSTANTS_MAX_ACCEL = DriveForward::MOTION_CONSTANTS_MAX_VELO / 0.5;

const double DriveForward::PID_CONSTANTS_KP = 0.01;//0.001;//75;
const double DriveForward::PID_CONSTANTS_KI = 0;
const double DriveForward::PID_CONSTANTS_KD = 0.000;

const double DriveForward::FF_CONSTANTS_S = 0.71922;
const double DriveForward::FF_CONSTANTS_V = 0.00625;//0.05;
const double DriveForward::FF_CONSTANTS_A = 0.0022; 

const double DriveForward::STRAIGHTEN_PID_KP = 0.115;//0.05;//0.033
const double DriveForward::STRAIGHTEN_PID_KI = 0;
const double DriveForward::STRAIGHTEN_PID_KD = 0.000;


void DriveForward::start(){ 

    startX = Telemetry::inst.getValueAt<double>("odometry", "x_position_mm"); 
    startY = Telemetry::inst.getValueAt<double>("odometry", "y_position_mm"); 
    
    initialAngle = Telemetry::inst.getValueAt<double>("odometry", "heading_deg");
     
    ffController.kS = FF_CONSTANTS_S; 
    ffController.kV = FF_CONSTANTS_V; 
    ffController.kA = FF_CONSTANTS_A;  
    
    PIDConstants pidConstants; 
    pidConstants.P = PID_CONSTANTS_KP; 
    pidConstants.I = PID_CONSTANTS_KI; 
    pidConstants.D = PID_CONSTANTS_KD;  
    
    PIDConstants straightenConstants; 
    straightenConstants.P = STRAIGHTEN_PID_KP; 
    straightenConstants.I = STRAIGHTEN_PID_KI; 
    straightenConstants.D = STRAIGHTEN_PID_KD;
     
    TrapezoidConstants motionConstants; 
    motionConstants.maxVelocity = MOTION_CONSTANTS_MAX_VELO * (percentVelo / 100.0); 
    motionConstants.maxAcceleration = MOTION_CONSTANTS_MAX_ACCEL * (percentAccel / 100.0); 

    controller = new pidcontroller(pidConstants, 0);   
    straightener = new pidcontroller(straightenConstants, 0); 

    motionProfile = new TrapezoidalMotionProfile(motionConstants, distance); 
    
    controller->setLastTimestamp(Brain.Timer.time()); 
    motionProfile->setLastTimestamp(Brain.Timer.time());    
    straightener->setLastTimestamp(Brain.Timer.time());

} 

void DriveForward::periodic(){    
    TrapezoidalSetpoint motionGoal = motionProfile->generateSetpoint(Brain.Timer.time());   

    double setpointVelocity = motionGoal.velocity; 
    double setpointAcceleration = motionGoal.acceleration;  

    //Telemetry::inst.placeValueAt<double>(setpointVelocity, "graph", "expected_velocity"); 
    //Telemetry::inst.placeValueAt<double>(setpointAcceleration, "graph", "expected_acceleration");
    //Telemetry::inst.placeValueAt<double>(Telemetry::inst.getValueAt<double>("odometry", "velocity_ms"), "graph", "current_velocity");

    double ffOutput = ffController.calculate(setpointVelocity, setpointAcceleration);  
    double correction = controller->calculate(Telemetry::inst.getValueAt<double>("odometry", "velocity_ms") - setpointVelocity, Brain.Timer.time()); 
    
    double turnCorrection = straightener->calculate(angleDifference(initialAngle, Telemetry::inst.getValueAt<double>("odometry", "heading_deg")), Brain.Timer.time());

    double output = ffOutput + correction;
    
    drivebaseRef.manualDrive(output, turnCorrection);

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

const double TurnToHeading::PID_CONSTANTS_KP = 12.0/105;
const double TurnToHeading::PID_CONSTANTS_KI = 0.001;//0.110;
const double TurnToHeading::PID_CONSTANTS_KD = 0.0015;

void TurnToHeading::start(){ 
   PIDConstants pidConstants;

   pidConstants.P = PID_CONSTANTS_KP; 
   pidConstants.I = PID_CONSTANTS_KI; 
   pidConstants.D = PID_CONSTANTS_KD;  

   pidConstants.errorTolerance = 3; 
     
   controller = new pidcontroller(pidConstants, 0); 
   controller->setLastTimestamp(Brain.Timer.time());   

} 

void TurnToHeading::periodic(){   
    double error = getError();
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
}
 
double TurnToHeading::getError(){  
    double error = angleDifference(setpoint, Telemetry::inst.getValueAt<double>("odometry", "heading_deg"));
    //Telemetry::inst.placeValueAt<double>(error, "graph", "error");
    return error; 
} 

void TurnToHeading::setAngle(double angle){ 
    setpoint = angle;
}

//----------------------------------------------------------------------------------------- 

void DriveToSetpoint::calibrateSetpoints_euc(double currentX, double currentY){ 
   double angle = angleSum(toDegrees(atan2(setpointY - currentY, setpointX - currentX)), 360);
   static_cast<TurnToHeading*>(commands.at(0))->setAngle(angle); 
} 

void DriveToSetpoint::calibrateSetpoints_man_xy(double currentX, double currentY, double currentAngle){ 
   double xDist = setpointX - currentX;  
   
   double vertAngle = setpointY - currentY > 0 ? 90 : 270;
   double horiAngle = xDist > 0 ? 0 : 180;  

   if (fabs(angleDifference(currentAngle, horiAngle)) > 90){ 
     horiAngle = angleSum(horiAngle, 180);
     xDist *= -1;
   } 

   static_cast<TurnToHeading*>(commands.at(0))->setAngle(horiAngle); 
   static_cast<DriveForward*>(commands.at(1))->setDistance(xDist); 
   static_cast<TurnToHeading*>(commands.at(2))->setAngle(vertAngle); 
  

} 

void DriveToSetpoint::calibrateSetpoints_man_yx(double currentX, double currentY, double currentAngle){ 
   double yDist = setpointY - currentY;
   
   double vertAngle = yDist > 0 ? 90 : 270;
   double horiAngle = setpointX - currentX > 0 ? 0 : 180;  

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
    double currentX = Telemetry::inst.getValueAt<double>("odometry", "x_position_mm"); 
    double currentY = Telemetry::inst.getValueAt<double>("odometry", "y_position_mm");  
    setDistance(hypot(currentX - targetX, currentY - targetY) - offset);
    DriveForward::start();
} 

//-------------------------------------------------------------------------- 

DriveTrajectory::DriveTrajectory(Drivebase& drivebase, double lookAheadDist, double maxVelocity, double maxAcceleration): 
    Command<Drivebase>(drivebase), 
    drivebaseRef(drivebase), 
    lDist(lookAheadDist), 
    maxVelo(maxVelocity), 
    maxAccel(maxAcceleration)
    { 
      PIDConstants pidConsts; 
      pidConsts.P = TurnToHeading::PID_CONSTANTS_KP; 
      pidConsts.I = TurnToHeading::PID_CONSTANTS_KI; 
      pidConsts.D = TurnToHeading::PID_CONSTANTS_KD; 
      turnController = new pidcontroller(pidConsts, 0);
    }

void DriveTrajectory::start(){ 
  initializePath(); //Creates the trajectory 
  int steps = static_cast<int>(trajectory->calculateLength() / lDist); 
  std::vector<Point> points;
  trajectory->generatePoints(points, steps);
  donkey = new PurePursuit(points, lDist);   

  turnController->setLastTimestamp(Brain.Timer.time()); 
}  

void DriveTrajectory::findNextSpeeds(double linearError, double angularError, double& linearSpeed, double& angularSpeed){ 
  angularSpeed = turnController->calculate(angularError, Brain.Timer.time());  
}

void DriveTrajectory::periodic(){   

  double linearError; 
  double angularError; 

  donkey->calculateError( 
       Telemetry::inst.getValueAt<double>("odometry", "x_position_mm"), 
       Telemetry::inst.getValueAt<double>("odometry", "y_position_mm"), 
       Telemetry::inst.getValueAt<double>("odometry", "heading_deg"), 
       linearError, 
       angularError 
  );

  double linearSpeed = 0; 
  double angularSpeed = 0;

  findNextSpeeds(linearError, angularError, linearSpeed, angularSpeed); 

  drivebaseRef.setSpeeds(linearSpeed, angularSpeed);
} 


void Toggle::start(){ 
    startTime = Brain.Timer.time(); 
} 

void Toggle::periodic(){ 
   int timePassed = Brain.Timer.time() - startTime; 
   if (timePassed < 400){ 
      drivebaseRef.manualDrive(-12, 0); 
   } else { 
      drivebaseRef.manualDrive(5, 0); 
   }
}  

bool Toggle::isOver(){ 
  return Brain.Timer.time() - startTime >= 800;
} 

void Toggle::end(){ 
  drivebaseRef.manualDrive(0,0);
}

