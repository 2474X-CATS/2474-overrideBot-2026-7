#include "drivebase.h"
#include "math.h"
#include "../architecture/telemetry.h" 
#include <algorithm>

//---Drivebase: SUBSYSTEM  

Drivebase* Drivebase::globalRef = nullptr; 

double Drivebase::DRIVE_WHEEL_RADIUS_MM = 76.2; 
double Drivebase::PITCH_TOLERANCE = 10; 

void Drivebase::declareLocations(){    
   locations[0] = new Location( 
   "native-matchloader_left", 
    TILE_SIZE_MM, 
    60, 
    45, 
    180, 
    30
   );  
   locations[1] = new Location( 
   "native-matchloader_right", 
    TILE_SIZE_MM*5, 
    60, 
    45, 
    180, 
    30
   );; 
   locations[2] = new Location( 
   "native-highgoal_left", 
    TILE_SIZE_MM, 
    TILE_SIZE_MM * 2, 
    30, 
    0, 
    30
   );
   locations[3] = new Location( 
   "native-highgoal_right", 
    TILE_SIZE_MM * 5, 
    TILE_SIZE_MM * 2, 
    30, 
    0, 
    30
   );
   locations[4] = new Location( 
   "native-mid", 
    TILE_SIZE_MM * 2 + 200, 
    TILE_SIZE_MM * 2 + 200, 
    30, 
    45, 
    30
   ); 
   locations[5] = new Location( 
   "native-low", 
    TILE_SIZE_MM * 3 + 400, 
    TILE_SIZE_MM * 2 + 200, 
    30, 
    315, 
    30
   );  
   locations[6] = new Location( 
   "park_near", 
    TILE_SIZE_MM * 3, 
    210, 
    TILE_SIZE_MM * 0.625, 
    180, 
    90
   ); 
   locations[7] = new Location( 
   "foreign_matchloader_right", 
    TILE_SIZE_MM * 5, 
    TILE_SIZE_MM * 6 - 60, 
    45, 
    0, 
    30
   ); 
   locations[8] = new Location( 
   "foreign_matchloader_left", 
    TILE_SIZE_MM * 1, 
    TILE_SIZE_MM * 6 - 60, 
    45, 
    0, 
    30
   );
   locations[9] = new Location( 
   "foreign_highgoal_left", 
    TILE_SIZE_MM, 
    TILE_SIZE_MM * 4, 
    30, 
    180, 
    30
   ); 
   locations[10] = new Location( 
   "foreign_highgoal_right", 
    TILE_SIZE_MM * 5, 
    TILE_SIZE_MM * 4, 
    30, 
    180, 
    30
   );  
   locations[11] = new Location( 
   "foreign_mid", 
    TILE_SIZE_MM * 3 + 400, 
    TILE_SIZE_MM * 3 + 400, 
    30, 
    225, 
    30
   ); 
   locations[12] = new Location( 
   "foreign_low", 
    TILE_SIZE_MM * 2 + 200, 
    TILE_SIZE_MM * 3 + 400, 
    30, 
    135, 
    30
   );
   locations[13] = new Location( 
   "park_far", 
    TILE_SIZE_MM * 3, 
    TILE_SIZE_MM * 6 - 210, 
    TILE_SIZE_MM * 0.625, 
    0, 
    90
   );  

   locations[14] = nullptr; 
   locations[15] = nullptr;  
   locations[16] = nullptr; 
   locations[17] = nullptr; 

}; 

void Drivebase::init()
{   

   //driveGyro.setTurnType(vex::turnType::left); 

   leftDriveMotors.setStopping(vex::brakeType::brake);
   rightDriveMotors.setStopping(vex::brakeType::brake);
   
   //driveGyro.setTurnType(vex::turnType::left);
   driveGyro.calibrate();
   while (driveGyro.isCalibrating())
   {
      vex::this_thread::yield();
   }

   driveGyro.setHeading(0, vex::rotationUnits::deg);

   powerPID.P = 0.75;
   powerPID.I = 0.015;
   powerPID.D = 0.25; 
   powerPID.iLimit = 1500;
   powerPID.errorTolerance = 1;
   //------------------------------
   turnPID.P = 0.75;
   turnPID.I = 0.1;
   turnPID.D = 0.05; 
   turnPID.iLimit = 180;
   turnPID.errorTolerance = 0.5;

   set<double>("Pos_X", startX + ROBOT_WIDTH_MM / 2);
   set<double>("Pos_Y", startY + ROBOT_LENGTH_MM / 2); 
   set<double>("Velocity_mm/20ms", 0);
   set<string>("Current_Location", "NONE");  
   set<double>("Max_Pitch", 0); 

   declareLocations();
};

void Drivebase::periodic()
{ 
   arcadeDrive((double)RobotState::getAxisState(AxisType::LEFT_VERTICAL), (double)RobotState::getAxisState(AxisType::RIGHT_HORIZONTAL));
}; 

void Drivebase::updateTelemetry()
{
   double x = get<double>("Pos_X");
   double y = get<double>("Pos_Y");  
   set<double>("Angle_Degrees_CCW", fmod(90 + -driveGyro.heading() + 360, 360)); 

   double rpmToDist = (DRIVE_WHEEL_RADIUS_MM * 2 * M_PI) * 0.02;
   double hypotenuse = ((leftDriveMotors.velocity(vex::velocityUnits::rpm) - rightDriveMotors.velocity(vex::velocityUnits::rpm)) / 2) / 60 * rpmToDist; // Times 0.02 because that is the time interval

   double angleRadians = get<double>("Angle_Degrees_CCW") * (2*M_PI) / 360;

   if (driveGyro.pitch(vex::rotationUnits::deg) > get<double>("Max_Pitch")) 
      set<double>("Max_Pitch", driveGyro.pitch(vex::rotationUnits::deg));  
   x += (hypotenuse * cos(angleRadians));
   y += (hypotenuse * sin(angleRadians));
   

   set<double>("Pos_X", x);
   set<double>("Pos_Y", y);     
   set<double>("Velocity_mm/20ms", hypotenuse);  
   
   Brain.Screen.printAt(20,100,"X: %f",get<double>("Pos_X"));  
   Brain.Screen.printAt(20,125,"Y: %f",get<double>("Pos_Y"));  
   Brain.Screen.printAt(20,150,"Angle Heading: %f", get<double>("Angle_Degrees_CCW"));    
   Brain.Screen.printAt(20, 175, "Maximum Pitch Val: %f", get<double>("Max_Pitch")); 
   
   set<string>("Current_Location","NONE");   
   
   for (int index = 0; index < 14; index ++){ 
      if (locations.at(index) != nullptr){ 
         Location* currentLocation = locations.at(index);
         if (currentLocation->isRobotVisiting()){  
            set<string>("Current_Location",currentLocation->getName());  
            break;
         }
      } else { 
         continue; 
      }
   }   

   Brain.Screen.printAt(20,200, get<string>("Current_Location").c_str());

   Telemetry::inst.placeValueAt<double>(driveFrontLeft.temperature(), "Motor_Temps","DriveFrontLeft"); 
   Telemetry::inst.placeValueAt<double>(driveFrontRight.temperature(), "Motor_Temps","DriveFrontRight"); 
   Telemetry::inst.placeValueAt<double>(driveMidLeft.temperature(), "Motor_Temps","DriveMidLeft"); 
   Telemetry::inst.placeValueAt<double>(driveMidRight.temperature(), "Motor_Temps","DriveMidRight"); 
   Telemetry::inst.placeValueAt<double>(driveBackLeft.temperature(), "Motor_Temps","DriveBackLeft"); 
   Telemetry::inst.placeValueAt<double>(driveBackRight.temperature(), "Motor_Temps","DriveBackRight"); 
  
   
   
}; 


Location* Drivebase::getLocation(int index)
{ 
  return locations.at(index); 
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
   if (turnDeg < 0){
     leftDriveMotors.spin(vex::directionType::rev);
     rightDriveMotors.spin(vex::directionType::rev);  
   } else { 
     leftDriveMotors.spin(vex::directionType::fwd);
     rightDriveMotors.spin(vex::directionType::fwd);
   }
};

void Drivebase::stop()
{
   leftDriveMotors.setVelocity(0, vex::percentUnits::pct);
   rightDriveMotors.setVelocity(0, vex::percentUnits::pct);
};

/*
void Drivebase::updateTileCoordinates()
{   
 
}
*/
PIDConstants Drivebase::getPowerPID()
{
   return this->powerPID;
};

PIDConstants Drivebase::getTurningPID()
{
   return this->turnPID;
};

void Drivebase::setSpeedFactor(double speedFactor){ 
   this->speedFactor = speedFactor;
}