#ifndef __DRIVEBASE_H_
#define __DRIVEBASE_H_

#include "../architecture/subsystem.h" 
#include "../helpers/pidcontroller.h" 
#include "../helpers/location.h" 
#include <set>   
#include "vex.h"
//#include "../helpers/location.h"


class Drivebase : public Subsystem
{
private:
  // Motor initialization
  static double DRIVE_WHEEL_RADIUS_MM; 
  static double PITCH_TOLERANCE; 
  
  vex::inertial driveGyro; 

  vex::motor driveFrontLeft;
  vex::motor driveMidLeft;
  vex::motor driveBackLeft; 

  vex::motor driveBackRight;
  vex::motor driveFrontRight;
  vex::motor driveMidRight;

  vex::motor_group leftDriveMotors;
  vex::motor_group rightDriveMotors; 

  //==========================================
  PIDConstants powerPID;
  PIDConstants turnPID;
  
  double startX, startY;
  double speedFactor = 1;   
  
  array<Location*, 14> locations; 

  void declareLocations(); 
  //Location* currentLocation = nullptr; 

protected:
  using Subsystem::set;

public:
  using Subsystem::get;
  using Subsystem::getFromInputs; 

  static Drivebase* globalRef; 

  Drivebase(double tileX, double tileY) : Subsystem(
                                       "drivebase",
                                        {(EntrySet){"Pos_X", EntryType::DOUBLE},
                                         (EntrySet){"Pos_Y", EntryType::DOUBLE},
                                         (EntrySet){"Angle_Degrees_CCW", EntryType::DOUBLE},  
                                         (EntrySet){"Velocity_mm/20ms", EntryType::DOUBLE},
                                         (EntrySet){"Current_Location", EntryType::STRING}, 
                                         (EntrySet){"Max_Pitch", EntryType::DOUBLE} 
                                        }
                                    ),    
                                    driveGyro(vex::inertial(vex::PORT20)), 
                                    driveFrontLeft(vex::motor(vex::PORT1, vex::ratio18_1, true)), 
                                    driveMidLeft(vex::motor(vex::PORT2, vex::ratio18_1, true)),  
                                    driveBackLeft(vex::motor(vex::PORT3, vex::ratio18_1)), 
                                    driveBackRight(vex::motor(vex::PORT8, vex::ratio18_1)), 
                                    driveFrontRight(vex::motor(vex::PORT10, vex::ratio18_1, true)),
                                    driveMidRight(vex::motor(vex::PORT9, vex::ratio18_1, true)),
                                    leftDriveMotors(vex::motor_group(driveFrontLeft, driveBackLeft, driveMidLeft)), 
                                    rightDriveMotors(vex::motor_group(driveFrontRight, driveBackRight, driveMidRight)),
                startX((tileX - 1) * TILE_SIZE_MM),  
                startY((tileY - 1) * TILE_SIZE_MM) { 
                  globalRef = this;
                };

  void init() override;
  void periodic() override;
  void updateTelemetry() override;
  void stop() override;  

  void arcadeDrive(double speed, double rotation);
  void manualDriveForward(double speedMM);
  void manualTurnClockwise(double turnDeg); 
  
  void setSpeedFactor(double speedFactor);  

  Location* getLocation(int index); 
  
  //void updateTileCoordinates();
   
  //void updateLocations();
  
  PIDConstants getTurningPID();

  PIDConstants getPowerPID();
};

#endif