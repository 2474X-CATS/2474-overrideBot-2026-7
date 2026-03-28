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

double Drivebase::MID_ALIGNER_LENGTH = 0;
double Drivebase::HIGH_ALIGNER_LENGTH = 0;

double Drivebase::MAX_RPM = 360;

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
        TILE_SIZE_MM * 2.75,
        TILE_SIZE_MM * 2.75,
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


Drivebase::Drivebase(double startX, double startY) : Subsystem(
                                                "drivebase",
                                                { 
                                                 (EntrySet){"Pos_X", EntryType::DOUBLE},
                                                 (EntrySet){"Pos_Y", EntryType::DOUBLE},
                                                 (EntrySet){"Angle_Degrees_CCW", EntryType::DOUBLE},
                                                 (EntrySet){"last_heading", EntryType::DOUBLE},
                                                 (EntrySet){"overheating", EntryType::BOOL}, 
                                                 (EntrySet){"inst_speed", EntryType::DOUBLE}
                                                }),
                                            encoderLinear(vex::rotation(vex::PORT9)),
                                            driveGyro(vex::inertial(vex::PORT16)),                  // Used to be port 16
                                            driveFrontLeft(vex::motor(vex::PORT1, vex::ratio6_1)), // Used to be 1
                                            driveMidLeft(vex::motor(vex::PORT2, vex::ratio6_1, true)),
                                            driveBackLeft(vex::motor(vex::PORT3, vex::ratio6_1)),   // Used to be 3
                                            driveFrontRight(vex::motor(vex::PORT4, vex::ratio6_1)), // Used to be 4
                                            driveMidRight(vex::motor(vex::PORT5, vex::ratio6_1, true)),
                                            driveBackRight(vex::motor(vex::PORT15, vex::ratio6_1)), // Used to be 15
                                            leftDriveMotors(vex::motor_group(driveFrontLeft, driveBackLeft, driveMidLeft)),
                                            rightDriveMotors(vex::motor_group(driveFrontRight, driveBackRight, driveMidRight))
  {
   globalRef = this;  

   encoderLinear.setPosition(0, vex::rotationUnits::rev);
   // encoderAngular.setPosition(0, vex::rotationUnits::rev);

   leftDriveMotors.setStopping(vex::brakeType::coast);
   rightDriveMotors.setStopping(vex::brakeType::coast);

   driveGyro.calibrate();

   while (driveGyro.isCalibrating())
   {
      vex::this_thread::yield();
   }

   //------------------------------

   correctivePID.P = 0.25;
   correctivePID.I = 0.0;
   correctivePID.D = 0.0;
   correctivePID.errorTolerance = 0;

   //-------------------------- > 
   
   turnPID.P = 2.4; 
   turnPID.I = 0.00; 
   turnPID.D = 0.001; 
   turnPID.errorTolerance = 0.5;

   //-------------------------- >

   trapConsts.maxVelocity = ((MAX_RPM * (2 * DRIVE_WHEEL_RADIUS_MM * M_PI)) / 60.0); // * (MAX_RPM / 600.0));
   trapConsts.maxAcceleration = trapConsts.maxVelocity;                               // Reach max speed in 0.9 seconds

   //--------------------------
   
   turningProfile = new TrapezoidalMotionProfile(trapConsts, TILE_SIZE_MM);  
   turningCont = new errorcontroller(correctivePID); 

   //--------------------------

   lastTimestamp = Brain.Timer.time(vex::sec); 
   turningProfile->setLastTimestamp(lastTimestamp * 1000);  
   turningCont->setLastTimestamp(lastTimestamp * 1000); 
};

void Drivebase::periodic()
{
   arcadeDrive(((double)RobotState::getAxisState(AxisType::LEFT_VERTICAL))*-1, ((double)RobotState::getAxisState(AxisType::RIGHT_HORIZONTAL)));
   
}

void Drivebase::updateTelemetry()
{
     

   double x = get<double>("Pos_X");
   double y = get<double>("Pos_Y");


   if (RobotState::getStateOf("ready"))
   {
      double angle;
      angle = driveGyro.angle(vex::rotationUnits::deg);

      set<double>("Angle_Degrees_CCW", transformAngle(angle)); 
      
      double deltaTime = Brain.Timer.time(vex::sec) - lastTimestamp; 

      double omega = angleDifference(angle, get<double>("last_heading"));  
     
      if (!RobotState::getStateOf("is_counterclockwise")){ 
         omega *= -1;
      }
      
      set<double>("Angular_Velocity", omega / deltaTime);

      leftDriveMotors.setStopping(vex::brakeType::brake);
      rightDriveMotors.setStopping(vex::brakeType::brake);

      double hypotenuse;
      // hypotenuse = -((encoderLinear.velocity(vex::velocityUnits::rpm) * 2 * M_PI * ENCODER_WHEEL_LIN_RADIUS_MM) / 60 * deltaTime);
      hypotenuse = ((leftDriveMotors.velocity(vex::velocityUnits::rpm) - rightDriveMotors.velocity(vex::velocityUnits::rpm)) / 2) * 2 * M_PI * DRIVE_WHEEL_RADIUS_MM / 60 * deltaTime * (MAX_RPM / 600);
      
      if (RobotState::getStateOf("is_drive_inverted"))
      {
         hypotenuse *= -1;
      }  

      set<double>("Instantaneous_Speed", fabs(hypotenuse) / deltaTime);

      double angleRadians = toRadians(transformAngle(get<double>("last_heading"))); // * (2 * M_PI) / 360;

          x += (hypotenuse * cos(angleRadians));
          y += (hypotenuse * sin(angleRadians));
      }
      
      set<double>("last_heading", angle);
   }

   set<double>("Pos_X", x);
   set<double>("Pos_Y", y);
   set<double>("inst_speed", hypotenuse);

   if (RobotState::getStateOf("k_calibrating"))
   {
      if (RobotState::getStateOf("calibrating"))
      {
         calibrate(calibratingWall);
         calibratingWall = Alignment_Structure::NONE;
         RobotState::manuallyModifyState("calibrating", false);
         RobotState::manuallyModifyState("k_calibrating", false);
      }
   }

   
   //Brain.Screen.printAt(20, 150, "Linear Velocity: %.2f", get<double>("Instantaneous_Speed"));
   //Brain.Screen.printAt(20, 175, "Angular Velocity: %.2f", get<double>("Angular_Velocity"));
   Brain.Screen.printAt(20, 200, "Angle Degrees: %.2f", get<double>("Angle_Degrees_CCW"));
   

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

   if (fabs(speed) < 2 && fabs(rotation) < 2)
   {
      stop();
      return;
   }

   speed = speed > 100 ? 100 : (speed < -100 ? -100 : speed);
   rotation = rotation > 100 ? 100 : (rotation < -100 ? -100 : rotation);

   speed = RobotState::getStateOf("is_drive_inverted") ? speed * -1 : speed;

   leftDriveMotors.setVelocity((speed + rotation), vex::percentUnits::pct);
   rightDriveMotors.setVelocity((speed - rotation), vex::percentUnits::pct);

   leftDriveMotors.spin(vex::directionType::rev);
   rightDriveMotors.spin(vex::directionType::fwd);
};

void Drivebase::manualDriveForward(double speedMM, double centerAngle)
{
   double currentAngle = get<double>("Angle_Degrees_CCW");

   if (!RobotState::getStateOf("is_drive_inverted"))
   {
      speedMM *= -1;
   }

   double netSpeed = speedMM / (DRIVE_WHEEL_RADIUS_MM * 2 * M_PI) * 60 * (600.0 / MAX_RPM);

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
   if (!RobotState::getStateOf("is_drive_inverted"))
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
   double rotationsPerMinutes = (((ROBOT_WIDTH_MM * M_PI) * (turnDeg / 360.0)) / (DRIVE_WHEEL_RADIUS_MM * 2 * M_PI)) * 60 * (600.0 / MAX_RPM);
   leftDriveMotors.setVelocity(rotationsPerMinutes, vex::velocityUnits::rpm);
   rightDriveMotors.setVelocity(rotationsPerMinutes, vex::velocityUnits::rpm);
   leftDriveMotors.spin(vex::directionType::fwd);
   rightDriveMotors.spin(vex::directionType::fwd);
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

void Drivebase::manualDriveWithCurvature(double speedMM, double turnDeg)
{
   if (!RobotState::getStateOf("is_drive_inverted"))
   {
      speedMM *= -1;
   }

   double netLinSpeed = speedMM / (DRIVE_WHEEL_RADIUS_MM * 2 * M_PI) * 60 * (600.0 / MAX_RPM);
   double netRotSpeed = (((ROBOT_WIDTH_MM * M_PI) * (turnDeg / 360.0)) / (DRIVE_WHEEL_RADIUS_MM * 2 * M_PI)) * 60 * (600.0 / MAX_RPM);

   leftDriveMotors.setVelocity(netLinSpeed - netRotSpeed, vex::velocityUnits::rpm);
   rightDriveMotors.setVelocity(netLinSpeed + netRotSpeed, vex::velocityUnits::rpm);
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
   driveGyro.setHeading(90, vex::rotationUnits::deg);
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
   data.pidConstants = correctivePID; 
   data.motionConstants = trapConsts; 
   data.maximumCentripetalAcceleration = pow(trapConsts.maxVelocity,2) / 1000.0; 
   return data;
}