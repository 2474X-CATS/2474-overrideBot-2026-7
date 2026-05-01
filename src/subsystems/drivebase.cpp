#include "drivebase.h"
#include "math.h"
#include "../architecture/telemetry.h"
#include <algorithm> 

#include "../utilities/functools.h"

//---Drivebase: SUBSYSTEM

Drivebase *Drivebase::globalRef = nullptr;

double Drivebase::ENCODER_WHEEL_ROT_RADIUS_MM = 69.85 / 2;
double Drivebase::ENCODER_WHEEL_LIN_RADIUS_MM = 25.4; 

double Drivebase::ENCODER_DIST_FROM_CENTER = 91.3417;

double Drivebase::DRIVE_WHEEL_RADIUS_MM = 3.25 * 25.4 / 2; // 3.25in diam

double Drivebase::MID_ALIGNER_LENGTH = -30;
double Drivebase::HIGH_ALIGNER_LENGTH = 0; 

double Drivebase::ROBOT_WHEEL_BASE = 10.24 * 25.4;

double Drivebase::MAX_RPM = 450; // Was 360

Location *Drivebase::locations[14] = {
    new Location(
        "native-matchloader_left",
        TILE_SIZE_MM,
        0,
        45,
        180,
        30),
    new Location(
        "native-matchloader_right",
        TILE_SIZE_MM * 5,
        0,
        45,
        180,
        30),
    new Location(
        "native-highgoal_left",
        TILE_SIZE_MM,
        TILE_SIZE_MM * 2,
        30,
        0,
        30),
    new Location(
        "native-highgoal_right",
        TILE_SIZE_MM * 5,
        TILE_SIZE_MM * 2,
        30,
        0,
        30),
    new Location(
        "native-mid",
        TILE_SIZE_MM * 2.75,//3 - (((226.25 / 2) * sqrt(2)) * cos(M_PI/4)), //TILE_SIZE * 2.75 for both
        TILE_SIZE_MM * 2.75,//3 - (((226.25 / 2) * sqrt(2)) * sin(M_PI/4)), 
        30,
        45,
        30),
    new Location(
        "native-low",
        TILE_SIZE_MM * 3.25,
        TILE_SIZE_MM * 2.75,
        30,
        315,
        30),
    new Location(
        "park_near",
        TILE_SIZE_MM * 3,
        210,
        TILE_SIZE_MM * 0.625,
        180,
        90),
    new Location(
        "foreign_matchloader_right",
        TILE_SIZE_MM * 5,
        TILE_SIZE_MM * 6,
        45,
        0,
        30),
    new Location(
        "foreign_matchloader_left",
        TILE_SIZE_MM * 1,
        TILE_SIZE_MM * 6,
        45,
        0,
        30),
    new Location(
        "foreign_highgoal_left",
        TILE_SIZE_MM,
        TILE_SIZE_MM * 4,
        30,
        180,
        30),
    new Location(
        "foreign_highgoal_right",
        TILE_SIZE_MM * 5,
        TILE_SIZE_MM * 4,
        30,
        180,
        30),
    new Location(
        "foreign_mid",
        TILE_SIZE_MM * 3.25,
        TILE_SIZE_MM * 3.25,
        30,
        225,
        30),
    new Location(
        "foreign_low",
        TILE_SIZE_MM * 2.75,
        TILE_SIZE_MM * 3.25,
        30,
        135,
        30),
    new Location(
        "park_far",
        TILE_SIZE_MM * 3,
        TILE_SIZE_MM * 6 - 210,
        TILE_SIZE_MM * 0.625,
        0,
        90)};

void Drivebase::init()
{

   //encoderLinear.setPosition(0, vex::rotationUnits::rev);
   //encoderAngular.setPosition(0, vex::rotationUnits::rev);

   leftDriveMotors.setStopping(vex::brakeType::coast);
   rightDriveMotors.setStopping(vex::brakeType::coast);

   driveGyro.calibrate(); 

   while (driveGyro.isCalibrating())
   {
      vex::this_thread::yield();
   }

   driveGyro.setHeading(90, vex::rotationUnits::deg);
   set<double>("last_heading", 90);
   
   setStartingPos(startX, startY);

   //------------------------------

   correctivePID.P = 0.35;
   correctivePID.I = 0.0;
   correctivePID.D = 0.0;
   correctivePID.errorTolerance = 0;

   //--------------------------  > 
   
   turnPID.P = 2.825;
   turnPID.I = 0.11;
   turnPID.D = 0;//  
   turnPID.minimumOutput = 12.5;
   turnPID.errorTolerance = 1;

   //-------------------------- >

   trapConsts.maxVelocity = ((MAX_RPM * (2 * DRIVE_WHEEL_RADIUS_MM * M_PI)) / 60.0); // * (MAX_RPM / 600.0));
   trapConsts.maxAcceleration = trapConsts.maxVelocity / 0.9;                               // Reach max speed in 0.9 seconds

   //--------------------------
 
   lastTimestamp = Brain.Timer.time(vex::sec);  
   startingTimestamp = lastTimestamp;
   
};

void Drivebase::periodic()
{  
   arcadeDrive(((double)RobotState::getAxisState(AxisType::LEFT_VERTICAL)), ((double)RobotState::getAxisState(AxisType::RIGHT_HORIZONTAL)));
} 

void Drivebase::updateTelemetry()
{
   double x = get<double>("Pos_X");
   double y = get<double>("Pos_Y"); 

   double angle = driveGyro.angle(vex::rotationUnits::deg);

   set<double>("Angle_Degrees_CCW", transformAngle(angle)); 
      
   double deltaTime = Brain.Timer.time(vex::sec) - lastTimestamp; 

   double omega = angleDifference(angle, get<double>("last_heading"));  
     
   set<double>("Angular_Velocity", omega / deltaTime);
    

   if (RobotState::getStateOf("odometry_enabled")) 
   {  
      odomLift.close();
      
      double hypotenuse;
      hypotenuse = ((encoderLinear.velocity(vex::velocityUnits::rpm) * 2 * M_PI * ENCODER_WHEEL_LIN_RADIUS_MM) / 60 * deltaTime);
      //hypotenuse = ((leftDriveMotors.velocity(vex::velocityUnits::rpm) - rightDriveMotors.velocity(vex::velocityUnits::rpm)) / 2) * 2 * M_PI * DRIVE_WHEEL_RADIUS_MM / 60 * deltaTime * (MAX_RPM / 600);
      
      if (RobotState::getStateOf("is_drive_inverted"))
      {
         hypotenuse *= -1;
      }  

      set<double>("Linear_Velocity", hypotenuse / deltaTime);

      double angleRadians = toRadians(transformAngle(get<double>("last_heading"))); // * (2 * M_PI) / 360;

      x += (hypotenuse * cos(angleRadians));
      y += (hypotenuse * sin(angleRadians)); 

      if (RobotState::getStateOf("k_calibrating")){
        if (RobotState::getStateOf("calibrating"))
        {
         calibrate(calibratingWall);
         calibratingWall = Alignment_Structure::NONE;
         RobotState::manuallyModifyState("calibrating", false);
         RobotState::manuallyModifyState("k_calibrating", false);
        }
      }
   } else { 
      odomLift.open(); 
   }

   set<double>("last_heading", angle);  

   if (hasScheduledSetpoint){ 
     x = scheduledSetpoint[0]; 
     y = scheduledSetpoint[1]; 
     hasScheduledSetpoint = false;
   } 
   
   set<double>("Pos_X", x);
   set<double>("Pos_Y", y);


   //Brain.Screen.printAt(20, 150, "Angle Degrees: %.2f", get<double>("Angle_Degrees_CCW"));
   //Brain.Screen.printAt(20, 175, "Pos X: %.2f", get<double>("Pos_X"));  
   //Brain.Screen.printAt(20, 200, "Pos Y: %.2f", get<double>("Pos_Y")); 
   lastTimestamp = Brain.Timer.time(vex::sec);
};

Location *Drivebase::getLocation(int index)
{
   return Drivebase::locations[index];
}

void Drivebase::arcadeDrive(double speed, double rotation)
{
   
   speed *= linearSpeedFactor;  
   rotation *= angularSpeedFactor;

   speed = speed > 100 ? 100 : (speed < -100 ? -100 : speed);
   rotation = rotation > 100 ? 100 : (rotation < -100 ? -100 : rotation);

   speed = RobotState::getStateOf("is_drive_inverted") ? speed * -1 : speed;

   //leftDriveMotors.setVelocity((speed - rotation), vex::percentUnits::pct);
   //rightDriveMotors.setVelocity((speed + rotation), vex::percentUnits::pct);
   
   leftDriveMotors.spin(vex::directionType::rev, 12 * ((speed - rotation) / 100), vex::voltageUnits::volt); 
   rightDriveMotors.spin(vex::directionType::fwd, 12 * ((speed + rotation) / 100), vex::voltageUnits::volt);
   

   //leftDriveMotors.spin(vex::directionType::rev);
   //rightDriveMotors.spin(vex::directionType::fwd);
};

void Drivebase::manualDriveForward(double speedMM, double centerAngle)
{
   double currentAngle = get<double>("Angle_Degrees_CCW");

   if (RobotState::getStateOf("is_drive_inverted"))
   {
      speedMM *= -1;
   }

   double netSpeed = speedMM / (DRIVE_WHEEL_RADIUS_MM * 2 * M_PI) * 60;
   netSpeed *= (600.0 / MAX_RPM);

   double angleCorrection = 0;

   if (centerAngle != -1)
   {
      angleCorrection = ((DRIVE_WHEEL_RADIUS_MM * 2 * M_PI) * (angleDifference(centerAngle, currentAngle) / 360.0)) * 3;
   }

   leftDriveMotors.setVelocity(netSpeed - angleCorrection, vex::velocityUnits::rpm);
   rightDriveMotors.setVelocity(netSpeed + angleCorrection, vex::velocityUnits::rpm);
   leftDriveMotors.spin(vex::directionType::rev);
   rightDriveMotors.spin(vex::directionType::fwd);
};

void Drivebase::manualPercentageDrive(double decimal)
{
   if (RobotState::getStateOf("is_drive_inverted"))
   {
      decimal *= -1;
   } 

   leftDriveMotors.setVelocity(decimal, vex::velocityUnits::pct);
   rightDriveMotors.setVelocity(decimal, vex::velocityUnits::pct);
   leftDriveMotors.spin(vex::directionType::rev);
   rightDriveMotors.spin(vex::directionType::fwd);
}

void Drivebase::manualTurnCounterclockwise(double turnDeg)
{
   double rotationsPerMinutes = (((ROBOT_WHEEL_BASE * M_PI) * (turnDeg / 360.0)) / (DRIVE_WHEEL_RADIUS_MM * 2 * M_PI)) * 60 * (600.0 / MAX_RPM);
   leftDriveMotors.setVelocity(rotationsPerMinutes, vex::velocityUnits::rpm);
   rightDriveMotors.setVelocity(rotationsPerMinutes, vex::velocityUnits::rpm);
   leftDriveMotors.spin(vex::directionType::rev);
   rightDriveMotors.spin(vex::directionType::rev);
};

double Drivebase::transformAngle(double heading)
{
   if (RobotState::getStateOf("is_drive_inverted"))
   {
      heading = angleSum(heading, 180);
   }

   if (!RobotState::getStateOf("is_counterclockwise"))
   {
      heading = flipOrientation(heading);
   }

   return heading;
} 

void Drivebase::scheduleSetpoint(double x, double y){ 
   scheduledSetpoint[0] = x; 
   scheduledSetpoint[1] = y;  
   hasScheduledSetpoint = true;
}

void Drivebase::initializePosWithFaces(double x, double y){ 
   double heading = toRadians(get<double>("Angle_Degrees_CCW"));
   set<double>("Pos_X", x + cos(heading) * (ROBOT_LENGTH_MM/2));  
   set<double>("Pos_Y", y + sin(heading) * (ROBOT_LENGTH_MM/2));
} 

void Drivebase::initializePosWithVertices(double x, double y){ 
   
   double heading = get<double>("Angle_Degrees_CCW");   
   
   double xComp;
   double yComp; 

   int referenceAngle;

   if ((heading >= 45 && heading < 135) || (heading >= 225 && heading < 315)){  
     if (heading >= 45 && heading < 135){ 
      referenceAngle = 90;
     } else { 
      referenceAngle = 270;
     }
     xComp = ROBOT_WIDTH_MM/2; 
     yComp = ROBOT_LENGTH_MM/2; 
   } else {   

     if (heading >= 135 && heading < 225){ 
      referenceAngle = 180;
     } else { 
      referenceAngle = 0;
     }  

     xComp = ROBOT_LENGTH_MM/2; 
     yComp = ROBOT_WIDTH_MM/2;
   } 

   double dist = hypot(xComp, yComp); 
    
   double standardAngle;

   switch (referenceAngle){ 
      case 90:  
        standardAngle = atan2(yComp, xComp); 
        break;
      case 180:
        standardAngle = atan2(yComp, -xComp); 
        break;
      case 270:  
        standardAngle = (2*M_PI) + atan2(-yComp, -xComp); 
        break;
      case 0:  
        standardAngle = (2*M_PI) + atan2(-yComp, xComp);  
        break; 
      default: 
        standardAngle = -1; 
   }  

   double angleDiff = angleDifference(referenceAngle, heading);  

   standardAngle -= toRadians(angleDiff);  
    
   set<double>("Pos_X", x + cos(standardAngle) * dist);
   set<double>("Pos_Y", y + sin(standardAngle) * dist);
   
}

void Drivebase::manualDriveWithCurvature(double speedMM, double turnDeg)
{ 

   if (RobotState::getStateOf("is_drive_inverted"))
   {
      speedMM *= -1;
   }

   double netLinSpeed = speedMM / (DRIVE_WHEEL_RADIUS_MM * 2 * M_PI) * 60 * (600.0 / MAX_RPM);
   double netRotSpeed = (((ROBOT_WIDTH_MM * M_PI) * (turnDeg / 360.0)) / (DRIVE_WHEEL_RADIUS_MM * 2 * M_PI)) * 60 * (600.0 / MAX_RPM);

   leftDriveMotors.setVelocity(netLinSpeed + netRotSpeed, vex::velocityUnits::rpm);
   rightDriveMotors.setVelocity(netLinSpeed - netRotSpeed, vex::velocityUnits::rpm); 

   leftDriveMotors.spin(vex::directionType::rev);
   rightDriveMotors.spin(vex::directionType::fwd);
}

void Drivebase::voltageDriveForward(double volts)
{
   if (!RobotState::getStateOf("is_drive_inverted"))
   {
      volts *= -1;
   }
   volts = volts > 12.0 ? 12.0 : (volts < -12.0 ? -12.0 : volts);
   leftDriveMotors.spin(vex::directionType::rev, volts, vex::voltageUnits::volt);
   rightDriveMotors.spin(vex::directionType::fwd, volts, vex::voltageUnits::volt);
};

void Drivebase::voltageTurnClockwise(double volts)
{
   volts = volts > 12.0 ? 12.0 : (volts < -12.0 ? -12.0 : volts);
   leftDriveMotors.spin(vex::directionType::fwd, volts, vex::voltageUnits::volt);
   rightDriveMotors.spin(vex::directionType::fwd, volts, vex::voltageUnits::volt);
};

void Drivebase::stop()
{
   leftDriveMotors.setVelocity(0, vex::percentUnits::pct);
   rightDriveMotors.setVelocity(0, vex::percentUnits::pct);
};

void Drivebase::calibrate(Alignment_Structure struc)
{
   double supposedAngle;
   Location *loc = nullptr;
   array<double, 2> calibrationPoint;
   switch (struc)
   {
   case LEFT_WALL:
      set<double>("Pos_X", ROBOT_LENGTH_MM / 2);
      supposedAngle = 180;
      break;
   case RIGHT_WALL:
      set<double>("Pos_X", (TILE_SIZE_MM * 6) - (ROBOT_LENGTH_MM / 2));
      supposedAngle = 0;
      break;
   case UP_WALL:
      set<double>("Pos_Y", (TILE_SIZE_MM * 6) - (ROBOT_LENGTH_MM / 2));
      supposedAngle = 90;
      break;
   case DOWN_WALL:
      set<double>("Pos_Y", ROBOT_LENGTH_MM / 2);
      supposedAngle = 270;
      break;
   case NEARBY_HIGH_LEFT:
      loc = getLocation(2);
      supposedAngle = loc->getPerfectEntranceAngle();
      calibrationPoint = loc->getProjectedSetpoint(ROBOT_LENGTH_MM / 2 + HIGH_ALIGNER_LENGTH);
      set<double>("Pos_X", calibrationPoint.at(0));
      set<double>("Pos_Y", calibrationPoint.at(1));
      break;
   case NEARBY_HIGH_RIGHT:
      loc = getLocation(3);
      supposedAngle = loc->getPerfectEntranceAngle();
      calibrationPoint = loc->getProjectedSetpoint(ROBOT_LENGTH_MM / 2 + HIGH_ALIGNER_LENGTH);
      set<double>("Pos_X", calibrationPoint.at(0));
      set<double>("Pos_Y", calibrationPoint.at(1));
      break;
   case FOREIGN_HIGH_LEFT:
      loc = getLocation(9);
      supposedAngle = loc->getPerfectEntranceAngle();
      calibrationPoint = loc->getProjectedSetpoint(ROBOT_LENGTH_MM / 2 + HIGH_ALIGNER_LENGTH);
      set<double>("Pos_X", calibrationPoint.at(0));
      set<double>("Pos_Y", calibrationPoint.at(1));
      break;
   case FOREIGN_HIGH_RIGHT:
      loc = getLocation(10);
      supposedAngle = loc->getPerfectEntranceAngle();
      calibrationPoint = loc->getProjectedSetpoint(ROBOT_LENGTH_MM / 2 + HIGH_ALIGNER_LENGTH);
      set<double>("Pos_X", calibrationPoint.at(0));
      set<double>("Pos_Y", calibrationPoint.at(1));
      break;
   case NEARBY_MID:
      loc = getLocation(4);
      supposedAngle = loc->getPerfectEntranceAngle();
      calibrationPoint = loc->getProjectedSetpoint(ROBOT_LENGTH_MM / 2 + MID_ALIGNER_LENGTH);
      set<double>("Pos_X", calibrationPoint.at(0));
      set<double>("Pos_Y", calibrationPoint.at(1));
      break;
   case NEARBY_LOW:
      loc = getLocation(5);
      supposedAngle = loc->getPerfectEntranceAngle();
      calibrationPoint = loc->getProjectedSetpoint(ROBOT_LENGTH_MM / 2 + MID_ALIGNER_LENGTH);
      set<double>("Pos_X", calibrationPoint.at(0));
      set<double>("Pos_Y", calibrationPoint.at(1));
      break;
   case FOREIGN_MID:
      loc = getLocation(11);
      supposedAngle = loc->getPerfectEntranceAngle();
      calibrationPoint = loc->getProjectedSetpoint(ROBOT_LENGTH_MM / 2 + MID_ALIGNER_LENGTH);
      set<double>("Pos_X", calibrationPoint.at(0));
      set<double>("Pos_Y", calibrationPoint.at(1));
      break;
   case FOREIGN_LOW:
      loc = getLocation(12);
      supposedAngle = loc->getPerfectEntranceAngle();
      calibrationPoint = loc->getProjectedSetpoint(ROBOT_LENGTH_MM / 2 + MID_ALIGNER_LENGTH);
      set<double>("Pos_X", calibrationPoint.at(0));
      set<double>("Pos_Y", calibrationPoint.at(1));
      break;
   case NONE:
   default:
      supposedAngle = -1;
      break;
   }

   if (supposedAngle == -1)
   {
      return;
   }

   if (!RobotState::getStateOf("is_counterclockwise"))
   {
      supposedAngle = flipOrientation(supposedAngle);
   }

   if (RobotState::getStateOf("is_drive_inverted"))
   {
      supposedAngle = angleSum(supposedAngle, 180);
   } 

   driveGyro.setHeading(supposedAngle, vex::rotationUnits::deg);
}

PIDConstants Drivebase::getTurningPID()
{
   return this->turnPID;
};

void Drivebase::setStartingPos(double x, double y)
{
   set<double>("Pos_X", x + (ROBOT_WIDTH_MM / 2));
   set<double>("Pos_Y", y + (ROBOT_LENGTH_MM / 2)); 
   leftDriveMotors.setStopping(vex::brakeType::brake);
   rightDriveMotors.setStopping(vex::brakeType::brake);
   //driveGyro.setHeading(90, vex::rotationUnits::deg);
}

TrapezoidConstants Drivebase::getMotionConstants()
{
   return this->trapConsts;
};

void Drivebase::setCalibratingStructure(Alignment_Structure struc)
{ 
   this->calibratingWall = struc;
}; 

PathMetadata Drivebase::getPathMetadata(){ 
   PathMetadata data; 
   data.positionX = get<double>("Pos_X"); 
   data.positionY = get<double>("Pos_Y"); 
   data.angleHeading = get<double>("Angle_Degrees_CCW"); 
   data.correctiveTurnConstants = correctivePID;  
   data.correctiveLinConstants = correctivePID;
   data.motionConstants = trapConsts; 
   data.maximumCentripetalAcceleration = pow(trapConsts.maxVelocity,2) / 500; 
   return data;
} 

PathFrame Drivebase::getFrameData(){ 
   PathFrame frame; 
   frame.positionX = get<double>("Pos_X"); 
   frame.positionY = get<double>("Pos_Y"); 
   frame.angleHeading = get<double>("Angle_Degrees_CCW"); 
   frame.angularVelocity = get<double>("Angular_Velocity"); 
   frame.linearVelocity = get<double>("Linear_Velocity"); 
   frame.timestamp = Brain.Timer.time();  
   return frame;
}  

PIDConstants Drivebase::getCorrectiveLinearPID(){ 
   return correctivePID;
}
