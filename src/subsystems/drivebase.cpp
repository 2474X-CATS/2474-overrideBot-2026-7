#include "drivebase.h"
#include "math.h"
#include "../architecture/telemetry.h"
#include <algorithm>

//---Drivebase: SUBSYSTEM

Drivebase *Drivebase::globalRef = nullptr;

double Drivebase::ENCODER_WHEEL_RADIUS_MM = 24.9;
double Drivebase::ENCODER_DIST_FROM_CENTER = 17.654 * 2; // 17.665
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
   encoderAngular.setPosition(0, vex::rotationUnits::rev);
   encoderLinear.setReversed(true);

   leftDriveMotors.setStopping(vex::brakeType::brake);
   rightDriveMotors.setStopping(vex::brakeType::brake);

   powerPID.P = 1.25;
   powerPID.I = 0.075;
   powerPID.D = 0.1;
   powerPID.iLimit = 750;
   powerPID.errorTolerance = 1;
   //------------------------------
   turnPID.P = 1.4;
   turnPID.I = 0.05;
   turnPID.D = 0;
   turnPID.iLimit = 180;
   turnPID.errorTolerance = 0.5;

   set<double>("Pos_X", startX - ROBOT_WIDTH_MM / 2);
   set<double>("Pos_Y", startY + ROBOT_LENGTH_MM / 2);
   set<double>("Angle_Degrees_CCW", 90);
   set<string>("Current_Location", "NONE");
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

   double rpmToDist = (2 * M_PI * ENCODER_WHEEL_RADIUS_MM);

   double angularVelocity;

   /*
   angularVelocity = -(rightDriveMotors.velocity(vex::velocityUnits::rpm) + leftDriveMotors.velocity(vex::velocityUnits::rpm)) / 2 / 60 / 50;
   angularVelocity *= (DRIVE_WHEEL_RADIUS_MM * 2 * M_PI);
   angularVelocity /= (ROBOT_WIDTH_MM * M_PI);
   angularVelocity *= 360;
   */

   angularVelocity = -(((encoderAngular.velocity(vex::velocityUnits::rpm) / 60 / 50) * rpmToDist) / (M_PI * ENCODER_DIST_FROM_CENTER)) * (180);

   currentAngle += angularVelocity;

   if (currentAngle > 360)
   {
      currentAngle = currentAngle - 360;
   }
   else if (currentAngle < 0)
   {
      currentAngle = 360 + currentAngle;
   }

   set<double>("Angle_Degrees_CCW", currentAngle);

   double hypotenuse = (encoderLinear.velocity(vex::velocityUnits::rpm) * rpmToDist) / 3000;

   double angleRadians = get<double>("Angle_Degrees_CCW") * (2 * M_PI) / 360;

   x += (hypotenuse * cos(angleRadians));
   y += (hypotenuse * sin(angleRadians));

   set<double>("Pos_X", x);
   set<double>("Pos_Y", y);

   set<string>("Current_Location", "NONE");

   for (int index = 0; index < 14; index++)
   {
      if (Drivebase::locations[index] != nullptr)
      {
         Location *currentLocation = Drivebase::locations[index];
         if (currentLocation->isRobotVisiting())
         {
            set<string>("Current_Location", currentLocation->getName());
            break;
         }
      }
      else
      {
         continue;
      }
   }

   Telemetry::inst.placeValueAt<double>(driveFrontLeft.temperature(), "Motor_Temps", "DriveFrontLeft");
   Telemetry::inst.placeValueAt<double>(driveFrontRight.temperature(), "Motor_Temps", "DriveFrontRight");
   Telemetry::inst.placeValueAt<double>(driveMidLeft.temperature(), "Motor_Temps", "DriveMidLeft");
   Telemetry::inst.placeValueAt<double>(driveMidRight.temperature(), "Motor_Temps", "DriveMidRight");
   Telemetry::inst.placeValueAt<double>(driveBackLeft.temperature(), "Motor_Temps", "DriveBackLeft");
   Telemetry::inst.placeValueAt<double>(driveBackRight.temperature(), "Motor_Temps", "DriveBackRight");

   //---------------------------------------------------------

   Brain.Screen.printAt(20, 100, "X: %f", get<double>("Pos_X"));
   Brain.Screen.printAt(20, 125, "Y: %f", get<double>("Pos_Y"));
   Brain.Screen.printAt(20, 150, "Angle Heading: %f", get<double>("Angle_Degrees_CCW"));
   // Brain.Screen.printAt(20,200, get<string>("Current_Location").c_str());
};

Location *Drivebase::getLocation(int index)
{
   return Drivebase::locations[index];
}

void Drivebase::arcadeDrive(double speed, double rotation)
{
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

void Drivebase::setSpeedFactor(double speedFactor)
{
   this->speedFactor = speedFactor;
}