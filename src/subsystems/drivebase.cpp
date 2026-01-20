#include "drivebase.h"
#include "math.h"
#include "../architecture/telemetry.h"
#include <algorithm>

//---Drivebase: SUBSYSTEM

Drivebase *Drivebase::globalRef = nullptr;

double Drivebase::ENCODER_WHEEL_ROT_RADIUS_MM = 69.85 / 2;  
double Drivebase::ENCODER_WHEEL_LIN_RADIUS_MM = 25.4;  
double Drivebase::ENCODER_DIST_FROM_CENTER = 91.3417; // 17.665
double Drivebase::DRIVE_WHEEL_RADIUS_MM = 69.85 / 2; 

double Drivebase::MID_ALIGNER_LENGTH = 0; 
double Drivebase::HIGH_ALIGNER_LENGTH = 0;

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

void Drivebase::init()
{  
   
   encoderLinear.setPosition(0, vex::rotationUnits::rev);
   encoderAngular.setPosition(0, vex::rotationUnits::rev);   

   leftDriveMotors.setStopping(vex::brakeType::coast);
   rightDriveMotors.setStopping(vex::brakeType::coast); 

   driveGyro.calibrate();  
   while (driveGyro.isCalibrating()){ 
      vex::this_thread::yield();
   }  

   driveGyro.setHeading(90, vex::rotationUnits::deg);
   
   //------------------------------ 

   turnPID.P = 2.6; 
   turnPID.I = 7.5;
   turnPID.D = 0.05; 
   turnPID.iLimit = 3.5;
   turnPID.errorTolerance = 1;
   
   //--------------------------  >
 
   trapConsts.maxVelocity = 1234;  
   trapConsts.maxAcceleration = 1234;  
   
   //-------------------------- 

   set<double>("Pos_X", startX + ROBOT_WIDTH_MM / 2);
   set<double>("Pos_Y", startY + ROBOT_LENGTH_MM / 2); 
   set<double>("Angle_Degrees_CCW", 90);     

   lastTimestamp = Brain.Timer.time(vex::sec);  
   
};

void Drivebase::periodic()
{        
   arcadeDrive(((double)RobotState::getAxisState(AxisType::LEFT_VERTICAL)), ((double)RobotState::getAxisState(AxisType::RIGHT_HORIZONTAL)));   
}

void Drivebase::updateTelemetry()
{     
   
   double x = get<double>("Pos_X");
   double y = get<double>("Pos_Y"); 
   
   double angle; 
   angle = driveGyro.angle(vex::rotationUnits::deg);  

   if (RobotState::getStateOf("is_drive_inverted")){ 
      angle += 180;
   }  
   
   angle = fmod(angle, 360); 

   set<double>("Angle_Degrees_CCW", angle);   

   if (!RobotState::getStateOf("ready")){   
    if (RobotState::getStateOf("configurable")){ 
       if (RobotState::getStateOf("starting_left")){ 
         x = (TILE_SIZE_MM * 2 + 430 - ROBOT_WIDTH_MM);
       } else { 
         x = (TILE_SIZE_MM * 3 + 200);
       } 
       x += (ROBOT_WIDTH_MM / 2);
       y = 425 + (ROBOT_LENGTH_MM / 2); 
    } 

   } else { 
      double deltaTime = Brain.Timer.time(vex::sec) - lastTimestamp;

      leftDriveMotors.setStopping(vex::brakeType::brake); 
      rightDriveMotors.setStopping(vex::brakeType::brake);  
   
      double hypotenuse; 
      hypotenuse = -((encoderLinear.velocity(vex::velocityUnits::rpm) * 2 * M_PI * ENCODER_WHEEL_LIN_RADIUS_MM) / 60 * deltaTime); 

      if (RobotState::getStateOf("is_drive_inverted")){ 
        hypotenuse *= -1;
      } 

      double angleRadians = get<double>("Angle_Degrees_CCW") * (2 * M_PI) / 360;

      x += (hypotenuse * cos(angleRadians));
      y += (hypotenuse * sin(angleRadians)); 
   }
     

   set<double>("Pos_X", x); 
   set<double>("Pos_Y", y);  
   
   double temperatureSum = 0;  
 
   temperatureSum += driveFrontLeft.temperature(vex::temperatureUnits::celsius); 
   temperatureSum += driveFrontRight.temperature(vex::temperatureUnits::celsius); 
   temperatureSum += driveMidLeft.temperature(vex::temperatureUnits::celsius); 
   temperatureSum += driveMidRight.temperature(vex::temperatureUnits::celsius); 
   temperatureSum += driveBackLeft.temperature(vex::temperatureUnits::celsius); 
   temperatureSum += driveBackRight.temperature(vex::temperatureUnits::celsius); 

   double avgTemp = temperatureSum / 6.0;
   
   set<bool>("overheating", avgTemp >= MOTOR_TEMP_LIMIT_CELSIUS);   


   if (RobotState::getStateOf("k_calibrating")){  
       if (RobotState::getStateOf("calibrating")){ 
         calibrate(calibratingWall); 
         calibratingWall = Alignment_Structure::NONE;
         RobotState::manuallyModifyState("calibrating", false);  
         RobotState::manuallyModifyState("k_calibrating", false);
       } 
   } 

   //---------------------------------------------------------
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

   if (fabs(speed) < 2 && fabs(rotation) < 2){
      stop(); 
      return; 
   } 

   speed = speed > 100 ? 100 : (speed < -100 ? -100 : speed);
   rotation = rotation > 100 ? 100 : (rotation < -100 ? -100 : rotation); 

   speed = RobotState::getStateOf("is_drive_inverted") ? speed * -1: speed;    

   leftDriveMotors.setVelocity((speed + rotation), vex::percentUnits::pct);
   rightDriveMotors.setVelocity((speed - rotation), vex::percentUnits::pct);  
   
   leftDriveMotors.spin(vex::directionType::rev);
   rightDriveMotors.spin(vex::directionType::fwd);  
   
};

void Drivebase::manualDriveForward(double speedMM)
{ 
   if (!RobotState::getStateOf("is_drive_inverted")){ 
      speedMM *= -1; 
   }
   double netSpeed = speedMM / (DRIVE_WHEEL_RADIUS_MM * 2 * M_PI) * 60;  
   netSpeed *= (4.0/3);   
   leftDriveMotors.setVelocity(netSpeed, vex::velocityUnits::rpm);
   rightDriveMotors.setVelocity(netSpeed, vex::velocityUnits::rpm);
   leftDriveMotors.spin(vex::directionType::rev);
   rightDriveMotors.spin(vex::directionType::fwd); 
};

void Drivebase::manualTurnClockwise(double turnDeg)
{ 
   double rotationsPerMinutes = (((ROBOT_WIDTH_MM * M_PI) * (turnDeg / 360.0)) / (DRIVE_WHEEL_RADIUS_MM * 2 * M_PI)) * 60;  
   rotationsPerMinutes *= (360.0 / (360 - 41)); 
   rotationsPerMinutes = rotationsPerMinutes > 450 ? 450 : (rotationsPerMinutes < -450 ? -450 : rotationsPerMinutes); 
   
   //rotationsPerMinutes = 0; 

   leftDriveMotors.setVelocity(rotationsPerMinutes, vex::velocityUnits::rpm);
   rightDriveMotors.setVelocity(rotationsPerMinutes, vex::velocityUnits::rpm);
   leftDriveMotors.spin(vex::directionType::fwd);
   rightDriveMotors.spin(vex::directionType::fwd);
}; 

void Drivebase::voltageDriveForward(double volts)
{ 
   if (!RobotState::getStateOf("is_drive_inverted")){ 
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

void Drivebase::calibrate(Alignment_Structure struc){  
   double supposedAngle;  
   Location* loc = nullptr; 
   array<double, 2> calibrationPoint;
   switch (struc){ 
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
      calibrationPoint = loc->getProjectedSetpoint(ROBOT_LENGTH_MM/2 + HIGH_ALIGNER_LENGTH); 
      set<double>("Pos_X", calibrationPoint.at(0)); 
      set<double>("Pos_Y", calibrationPoint.at(1));
      break; 
     case NEARBY_HIGH_RIGHT:  
      loc = getLocation(3); 
      supposedAngle = loc->getPerfectEntranceAngle();
      calibrationPoint = loc->getProjectedSetpoint(ROBOT_LENGTH_MM/2 + HIGH_ALIGNER_LENGTH); 
      set<double>("Pos_X", calibrationPoint.at(0)); 
      set<double>("Pos_Y", calibrationPoint.at(1));  
      break;
     case FOREIGN_HIGH_LEFT:  
      loc = getLocation(9); 
      supposedAngle = loc->getPerfectEntranceAngle();
      calibrationPoint = loc->getProjectedSetpoint(ROBOT_LENGTH_MM/2 + HIGH_ALIGNER_LENGTH); 
      set<double>("Pos_X", calibrationPoint.at(0)); 
      set<double>("Pos_Y", calibrationPoint.at(1));  
      break;
     case FOREIGN_HIGH_RIGHT:  
      loc = getLocation(10); 
      supposedAngle = loc->getPerfectEntranceAngle();
      calibrationPoint = loc->getProjectedSetpoint(ROBOT_LENGTH_MM/2 + HIGH_ALIGNER_LENGTH); 
      set<double>("Pos_X", calibrationPoint.at(0)); 
      set<double>("Pos_Y", calibrationPoint.at(1)); 
      break; 
     case NEARBY_MID: 
      loc = getLocation(4); 
      supposedAngle = loc->getPerfectEntranceAngle();
      calibrationPoint = loc->getProjectedSetpoint(ROBOT_LENGTH_MM/2 + MID_ALIGNER_LENGTH); 
      set<double>("Pos_X", calibrationPoint.at(0)); 
      set<double>("Pos_Y", calibrationPoint.at(1)); 
      break; 
     case NEARBY_LOW:  
      loc = getLocation(5);
      supposedAngle = loc->getPerfectEntranceAngle();
      calibrationPoint = loc->getProjectedSetpoint(ROBOT_LENGTH_MM/2 + MID_ALIGNER_LENGTH); 
      set<double>("Pos_X", calibrationPoint.at(0)); 
      set<double>("Pos_Y", calibrationPoint.at(1));   
      break;
     case FOREIGN_MID: 
      loc = getLocation(11);
      supposedAngle = loc->getPerfectEntranceAngle();
      calibrationPoint = loc->getProjectedSetpoint(ROBOT_LENGTH_MM/2 + MID_ALIGNER_LENGTH); 
      set<double>("Pos_X", calibrationPoint.at(0)); 
      set<double>("Pos_Y", calibrationPoint.at(1));   
      break;
     case FOREIGN_LOW: 
      loc = getLocation(12);
      supposedAngle = loc->getPerfectEntranceAngle();
      calibrationPoint = loc->getProjectedSetpoint(ROBOT_LENGTH_MM/2 + MID_ALIGNER_LENGTH); 
      set<double>("Pos_X", calibrationPoint.at(0)); 
      set<double>("Pos_Y", calibrationPoint.at(1));   
      break;
     case NONE:    
     default:
      supposedAngle = -1; 
      break;
   }   

   if (supposedAngle == -1){ 
      return; 
   } 

   if (RobotState::getStateOf("is_drive_inverted")){ 
      supposedAngle += 180;
   }  

   supposedAngle = fmod(supposedAngle, 360);
   
   driveGyro.setHeading(supposedAngle, vex::rotationUnits::deg); 
} 

PIDConstants Drivebase::getTurningPID()
{
   return this->turnPID;
}; 

TrapezoidConstants Drivebase::getMotionConstants(){ 
   return this->trapConsts; 
}; 

void Drivebase::setCalibratingStructure(Alignment_Structure struc){ 
   this->calibratingWall = struc;
};