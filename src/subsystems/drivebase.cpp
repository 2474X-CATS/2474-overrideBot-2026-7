#include "drivebase.h"
#include "math.h"
#include "../architecture/telemetry.h"
#include <algorithm>

//---Drivebase: SUBSYSTEM

Drivebase *Drivebase::globalRef = nullptr;

double Drivebase::ENCODER_WHEEL_RADIUS_MM = 25.4;
double Drivebase::ENCODER_DIST_FROM_CENTER = 19.02231081; // 17.665
double Drivebase::DRIVE_WHEEL_RADIUS_MM = 76.2;

// array<Location*, 14> Drivebase::locations;

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
   encoderLinear.setReversed(true); 

   
   driveGyro.calibrate();  

   while (driveGyro.isCalibrating()){ 
      vex::this_thread::yield(); 
   } 
  
  
   leftDriveMotors.setStopping(vex::brakeType::brake);
   rightDriveMotors.setStopping(vex::brakeType::brake);

   powerPID.P = 0.75;
   powerPID.I = 0.01;
   powerPID.D = 0.0;
   powerPID.errorTolerance = 1;
   //------------------------------
   turnPID.P = 2.5;
   turnPID.I = 0.01;
   turnPID.D = 0.05;
   //turnPID.iLimit = 270;
   turnPID.errorTolerance = 1; 
   //-------------------------- 
   ffConsts.kS = 0.1; 
   ffConsts.kV = 0.2; 
   ffConsts.kA = 0.3; 
   //-------------------------- 
   trapConsts.maxVelocity = 1000; 
   trapConsts.maxAcceleration = 10;

   set<double>("Pos_X", startX + ROBOT_WIDTH_MM / 2);
   set<double>("Pos_Y", startY + ROBOT_LENGTH_MM / 2);
   set<double>("Angle_Degrees_CCW", 90); 
   set<double>("Last_Velocity", 0);
  
};

void Drivebase::periodic()
{ 
   arcadeDrive(((double)RobotState::getAxisState(AxisType::LEFT_VERTICAL)), ((double)RobotState::getAxisState(AxisType::RIGHT_HORIZONTAL)));
}

void Drivebase::updateTelemetry()
{
   double x = get<double>("Pos_X");
   double y = get<double>("Pos_Y");
   double currentAngle = get<double>("Angle_Degrees_CCW");

   currentAngle = 90 - driveGyro.heading(vex::rotationUnits::deg);

   if (currentAngle < 0)
   {
      currentAngle += 360;
   } 

   set<double>("Angle_Degrees_CCW", currentAngle);

   double hypotenuse; 
   //hypotenuse = ((encoderLinear.velocity(vex::velocityUnits::rpm) * 2 * M_PI * ENCODER_WHEEL_RADIUS_MM) / 3000);
   hypotenuse = ((leftDriveMotors.velocity(vex::velocityUnits::rpm) + rightDriveMotors.velocity(vex::velocityUnits::rpm)) / 2 / 3000 * (2 * M_PI * DRIVE_WHEEL_RADIUS_MM)); 
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
   
   /*
   double currentVelocity = encoderLinear.velocity(vex::velocityUnits::rpm) / 60 * 2 * M_PI * ENCODER_WHEEL_RADIUS_MM; 
   double currentAcceleration = (currentVelocity - get<double>("Last_Velocity")) / 0.02;
   
   totalEntries++; 

   accumulatedVelocity += currentVelocity; 
   accumulatedAcceleration += currentAcceleration; 
   accumulatedVoltage += (leftDriveMotors.voltage() + rightDriveMotors.voltage()) / 2 / 0.02;

   Brain.Screen.printAt(20, 200, "AVG Acceleration (MM/S^2): %f", accumulatedAcceleration / totalEntries); 
   Brain.Screen.printAt(20, 175, "AVG Velocity (MM/S): %f", accumulatedVelocity / totalEntries);   
   Brain.Screen.printAt(20, 225, "AVG Voltage Applied (V/S): %f", accumulatedVoltage / totalEntries);
   set<double>("Last_Velocity", currentVelocity); 
   */
   
};

Location *Drivebase::getLocation(int index)
{
   return Drivebase::locations[index];
}

void Drivebase::arcadeDrive(double speed, double rotation)
{ 
   if (fabs(speed) < 2 && fabs(rotation) < 2){
      stop(); 
      return; 
   }
   speed = speed > 100 ? 100 : (speed < -100 ? -100 : speed);
   rotation = rotation > 100 ? 100 : (rotation < -100 ? -100 : rotation);
   leftDriveMotors.setVelocity((speed + rotation) * speedFactor, vex::percentUnits::pct);
   leftDriveMotors.spin(vex::directionType::fwd);
   rightDriveMotors.setVelocity((speed - rotation) * speedFactor, vex::percentUnits::pct);
   rightDriveMotors.spin(vex::directionType::rev);
};

void Drivebase::manualDriveForward(double speedMM)
{
   double netSpeed = speedMM / (DRIVE_WHEEL_RADIUS_MM * 2 * M_PI) * 60;
   leftDriveMotors.setVelocity(netSpeed, vex::velocityUnits::rpm);
   rightDriveMotors.setVelocity(-netSpeed, vex::velocityUnits::rpm);
   leftDriveMotors.spin(vex::directionType::fwd);
   rightDriveMotors.spin(vex::directionType::fwd); 
};

void Drivebase::manualTurnClockwise(double turnDeg)
{
   double rotationsPerMinutes = fabs((((ROBOT_LENGTH_MM * M_PI) * (turnDeg / 360.0)) / (DRIVE_WHEEL_RADIUS_MM * 2 * M_PI)) * 60);
   leftDriveMotors.setVelocity(rotationsPerMinutes, vex::velocityUnits::rpm);
   rightDriveMotors.setVelocity(rotationsPerMinutes, vex::velocityUnits::rpm);
   if (turnDeg < 0)
   {
      leftDriveMotors.spin(vex::directionType::rev);
      rightDriveMotors.spin(vex::directionType::rev);
   }
   else
   {
      leftDriveMotors.spin(vex::directionType::fwd);
      rightDriveMotors.spin(vex::directionType::fwd);
   }
};

void Drivebase::stop()
{
   leftDriveMotors.setVelocity(0, vex::percentUnits::pct);
   rightDriveMotors.setVelocity(0, vex::percentUnits::pct);
};

PIDConstants Drivebase::getPowerPID()
{
   return this->powerPID;
};

PIDConstants Drivebase::getTurningPID()
{
   return this->turnPID;
}; 

TrapezoidConstants Drivebase::getMotionConstants(){ 
   return this->trapConsts; 
}; 


void Drivebase::setSpeedFactor(double speedFactor)
{
   this->speedFactor = speedFactor;
}