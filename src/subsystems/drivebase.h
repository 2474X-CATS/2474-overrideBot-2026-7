#ifndef __DRIVEBASE_H_
#define __DRIVEBASE_H_

#include "../architecture/subsystem.h"
#include "../helpers/pidcontroller.h"  

#include "../helpers/trapezoidalMotion.h" 
#include "../helpers/feedForward.h" 

#include "../helpers/location.h"
#include <set>
#include "vex.h"


class Drivebase : public Subsystem
{
private:
  static double ENCODER_WHEEL_RADIUS_MM;
  //static double ENCODER_DIST_FROM_CENTER;
  static double DRIVE_WHEEL_RADIUS_MM; 

  vex::rotation encoderLinear;
  vex::inertial driveGyro;

  vex::motor driveFrontLeft;
  vex::motor driveMidLeft;
  vex::motor driveBackLeft;
  
  vex::motor driveFrontRight; 
  vex::motor driveMidRight; 
  vex::motor driveBackRight;

  vex::motor_group leftDriveMotors;
  vex::motor_group rightDriveMotors; 
  
  /*
  vex::motor testMotor8; 
  vex::motor testMotor7;  
  */ 

  //==========================================
  
  PIDConstants turnPID;  

  TrapezoidConstants trapConsts; 

  double startX, startY;
  double linearSpeedFactor = 1; 
  double angularSpeedFactor = 0.8; 

  static Location *locations[]; 
  
  /*
  USED FOR FINDING FEED FORWARD VALUES
  */ 

protected:
  using Subsystem::set;

public:
  using Subsystem::get;

  static Drivebase *globalRef;

  Drivebase(double tileX, double tileY) : Subsystem(
                                              "drivebase",
                                              {
                                                  (EntrySet){"Pos_X", EntryType::DOUBLE},
                                                  (EntrySet){"Pos_Y", EntryType::DOUBLE},
                                                  (EntrySet){"Angle_Degrees_CCW", EntryType::DOUBLE}, 
                                                  (EntrySet){"overheating", EntryType::BOOL}
                                              }), 

                                          encoderLinear(vex::rotation(vex::PORT15)),
                                          driveGyro(vex::PORT20), 
                                          
                                          driveFrontLeft(vex::motor(vex::PORT1, vex::ratio18_1)),
                                          driveMidLeft(vex::motor(vex::PORT2, vex::ratio18_1, true)),
                                          driveBackLeft(vex::motor(vex::PORT3, vex::ratio18_1)), 
                                          
                                          driveFrontRight(vex::motor(vex::PORT4, vex::ratio18_1)), 
                                          driveMidRight(vex::motor(vex::PORT5, vex::ratio18_1, true)),
                                          driveBackRight(vex::motor(vex::PORT6, vex::ratio18_1)),
                                           
                                          leftDriveMotors(vex::motor_group(driveFrontLeft, driveMidLeft, driveBackLeft)),
                                          rightDriveMotors(vex::motor_group(driveFrontRight, driveMidRight, driveBackRight)), 
                                          
                                          //testMotor7(vex::motor(vex::PORT7)), 
                                          //testMotor8(vex::motor(vex::PORT8)), 

                                          startX((tileX) * TILE_SIZE_MM),
                                          startY((tileY) * TILE_SIZE_MM)
  {
    globalRef = this;
  };

  void init() override;
  void periodic() override;
  void updateTelemetry() override;
  void stop() override;

  void arcadeDrive(double speed, double rotation); 

  void manualDriveForward(double speedMM);
  void manualTurnClockwise(double turnDeg); 
  
  void voltageDriveForward(double volts); 
  void voltageTurnClockwise(double volts);   

  static Location *getLocation(int index);

  PIDConstants getTurningPID();

  TrapezoidConstants getMotionConstants();  
  
};

#endif