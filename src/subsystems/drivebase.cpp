#include "drivebase.h"
#include "math.h"
#include "../architecture/telemetry.h"
#include <algorithm>

//---Drivebase: SUBSYSTEM

Drivebase *Drivebase::globalRef = nullptr;

double Drivebase::ENCODER_WHEEL_ROT_RADIUS_MM = 69.85;  
double Drivebase::ENCODER_WHEEL_LIN_RADIUS_MM = 25.4;  
double Drivebase::ENCODER_DIST_FROM_CENTER = 91.3417 / 2; // 17.665
double Drivebase::DRIVE_WHEEL_RADIUS_MM = 69.85;

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

   leftDriveMotors.setStopping(vex::brakeType::brake);
   rightDriveMotors.setStopping(vex::brakeType::brake); 

   driveGyro.calibrate(); 
   while (driveGyro.isCalibrating()){ 
      vex::this_thread::yield();
   }
   
   //------------------------------
   turnPID.P = 2.8;
   turnPID.I = 1;
   turnPID.D = 0.05;
   turnPID.iLimit = 2;
   turnPID.errorTolerance = 0.25; 
   //--------------------------  
 
   trapConsts.maxVelocity = 2750;  
   trapConsts.maxAcceleration = 1500;  
   
   //-------------------------- 

   set<double>("Pos_X", startX + ROBOT_WIDTH_MM / 2);
   set<double>("Pos_Y", startY + ROBOT_LENGTH_MM / 2);
   set<double>("Angle_Degrees_CCW", 90);  
};

void Drivebase::periodic()
{        
   arcadeDrive(((double)RobotState::getAxisState(AxisType::LEFT_VERTICAL)), ((double)RobotState::getAxisState(AxisType::RIGHT_HORIZONTAL)));
}

void Drivebase::updateTelemetry()
{  
   double x = get<double>("Pos_X");
   double y = get<double>("Pos_Y");
   
   double currentAngle; 
   //currentAngle = 90 - fmod((encoderAngular.position(vex::rotationUnits::rev) * 2 * ENCODER_WHEEL_ROT_RADIUS_MM * M_PI) / (2 * ENCODER_DIST_FROM_CENTER * M_PI) * 360, 360);  
   currentAngle = fmod((90 - driveGyro.angle(vex::rotationUnits::deg)) + 360, 360); 

   if (RobotState::getStateOf("is_drive_inverted")){ 
      currentAngle += 180;
   }  
   
   if (currentAngle < 0)
   {
      currentAngle += 360;
   } else if (currentAngle > 360){ 
      currentAngle -= 360; 
   }

   set<double>("Angle_Degrees_CCW", currentAngle); 
   
   double hypotenuse; 
   hypotenuse = -((encoderLinear.velocity(vex::velocityUnits::rpm) * 2 * M_PI * ENCODER_WHEEL_LIN_RADIUS_MM) / 3000); 

   if (RobotState::getStateOf("is_drive_inverted")){ 
      hypotenuse *= -1;
   } 

   double angleRadians = get<double>("Angle_Degrees_CCW") * (2 * M_PI) / 360;

   x += (hypotenuse * cos(angleRadians));
   y += (hypotenuse * sin(angleRadians));

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

   //---------------------------------------------------------
   
   Brain.Screen.printAt(20, 100, "X: %f", get<double>("Pos_X"));
   Brain.Screen.printAt(20, 125, "Y: %f", get<double>("Pos_Y")); 
   Brain.Screen.printAt(20, 150, "Angle Heading: %f", get<double>("Angle_Degrees_CCW"));  
 
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
   if (RobotState::getStateOf("is_drive_inverted")){ 
      speedMM *= -1; 
   }
   double netSpeed = speedMM / (DRIVE_WHEEL_RADIUS_MM * 2 * M_PI) * 60;   
   leftDriveMotors.setVelocity(netSpeed, vex::velocityUnits::rpm);
   rightDriveMotors.setVelocity(-netSpeed, vex::velocityUnits::rpm);
   leftDriveMotors.spin(vex::directionType::fwd);
   rightDriveMotors.spin(vex::directionType::fwd); 
};

void Drivebase::manualTurnClockwise(double turnDeg)
{ 
   double rotationsPerMinutes = (((ROBOT_LENGTH_MM * M_PI) * (turnDeg / 360.0)) / (DRIVE_WHEEL_RADIUS_MM * 2 * M_PI)) * 60;
   leftDriveMotors.setVelocity(rotationsPerMinutes, vex::velocityUnits::rpm);
   rightDriveMotors.setVelocity(rotationsPerMinutes, vex::velocityUnits::rpm);
   leftDriveMotors.spin(vex::directionType::fwd);
   rightDriveMotors.spin(vex::directionType::fwd);
}; 

void Drivebase::voltageDriveForward(double volts)
{ 
   if (RobotState::getStateOf("is_drive_inverted")){ 
      volts *= -1; 
   }
   volts = volts > 12.0 ? 12.0 : (volts < -12.0 ? -12.0 : volts);
   leftDriveMotors.setVelocity((volts / 12.0) * 100.0, vex::velocityUnits::pct);
   rightDriveMotors.setVelocity(-(volts / 12.0) * 100.0, vex::velocityUnits::pct);
   leftDriveMotors.spin(vex::directionType::fwd);
   rightDriveMotors.spin(vex::directionType::fwd); 
};

void Drivebase::voltageTurnClockwise(double volts)
{
   volts = volts > 12.0 ? 12.0 : (volts < -12.0 ? -12.0 : volts);
   leftDriveMotors.setVelocity((volts / 12.0) * 100.0, vex::velocityUnits::pct);
   rightDriveMotors.setVelocity((volts / 12.0) * 100.0, vex::velocityUnits::pct);
   leftDriveMotors.spin(vex::directionType::fwd);
   rightDriveMotors.spin(vex::directionType::fwd);
};

void Drivebase::stop()
{
   leftDriveMotors.setVelocity(0, vex::percentUnits::pct);
   rightDriveMotors.setVelocity(0, vex::percentUnits::pct);
};


PIDConstants Drivebase::getTurningPID()
{
   return this->turnPID;
}; 

TrapezoidConstants Drivebase::getMotionConstants(){ 
   return this->trapConsts; 
}; 

