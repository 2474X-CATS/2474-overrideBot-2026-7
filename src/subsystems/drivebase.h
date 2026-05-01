#ifndef __DRIVEBASE_H_
#define __DRIVEBASE_H_

#include "../architecture/subsystem.h"
#include "../control/pidcontroller.h"

#include "../control/trapezoidalMotion.h"
#include "../utilities/location.h"

#include "../control/path.h"

#include <set>
#include "vex.h"

typedef enum
{

  LEFT_WALL,
  RIGHT_WALL,
  DOWN_WALL,
  UP_WALL,

  NEARBY_HIGH_LEFT,
  NEARBY_HIGH_RIGHT,
  FOREIGN_HIGH_LEFT,
  FOREIGN_HIGH_RIGHT,

  NEARBY_MID,
  NEARBY_LOW,
  FOREIGN_MID,
  FOREIGN_LOW,

  NONE

} Alignment_Structure;

class Drivebase : public Subsystem
{
private:
  static double ENCODER_WHEEL_ROT_RADIUS_MM;
  static double ENCODER_WHEEL_LIN_RADIUS_MM;
  static double ENCODER_DIST_FROM_CENTER;
  static double DRIVE_WHEEL_RADIUS_MM;
  static double MID_ALIGNER_LENGTH;
  static double HIGH_ALIGNER_LENGTH;
  static double MAX_RPM; 
  static double ROBOT_WHEEL_BASE;

  vex::rotation encoderLinear;
  // vex::rotation encoderAngular;

  vex::inertial driveGyro;

  vex::motor driveFrontLeft;
  vex::motor driveMidLeft;
  vex::motor driveBackLeft;

  vex::motor driveFrontRight;
  vex::motor driveMidRight;
  vex::motor driveBackRight; 

  vex::pneumatics odomLift; 

  vex::motor_group leftDriveMotors;
  vex::motor_group rightDriveMotors;
   
  PIDConstants turnPID;  

  PIDConstants correctivePID;

  TrapezoidConstants trapConsts; 

  double startX, startY;
  double linearSpeedFactor = 1;
  double angularSpeedFactor = 1; 

  bool hasScheduledSetpoint = false; 
  double scheduledSetpoint[2]; 

  double lastTimestamp = 0; 
  double startingTimestamp = 0;

  Alignment_Structure calibratingWall = Alignment_Structure::NONE;

  static Location *locations[];

  void calibrate(Alignment_Structure struc);

  double transformAngle(double heading);

protected:
  using Subsystem::set;

public:
  using Subsystem::get;

  static Drivebase *globalRef;

  Drivebase(double startX, double startY) : Subsystem(
                                                "drivebase",
                                                {(EntrySet){"Pos_X", EntryType::DOUBLE},
                                                 (EntrySet){"Pos_Y", EntryType::DOUBLE},
                                                 (EntrySet){"Angle_Degrees_CCW", EntryType::DOUBLE},
                                                 (EntrySet){"last_heading", EntryType::DOUBLE},
                                                 (EntrySet){"overheating", EntryType::BOOL}, 
                                                 (EntrySet){"Angular_Velocity", EntryType::DOUBLE}, 
                                                 (EntrySet){"Linear_Velocity", EntryType::DOUBLE}
                                                }),
                                            
                                            encoderLinear(vex::rotation(vex::PORT11)),
                                            //encoderAngular(vex::rotation(vex::PORT10)), 
                                            driveGyro(vex::inertial(vex::PORT12)), // Used to be port 16 

                                            driveFrontLeft(vex::motor(vex::PORT3, vex::ratio6_1)), // Used to be 1
                                            driveMidLeft(vex::motor(vex::PORT1, vex::ratio6_1)),
                                            driveBackLeft(vex::motor(vex::PORT2, vex::ratio6_1, true)),   // Used to be 3
                                            
                                            driveFrontRight(vex::motor(vex::PORT6, vex::ratio6_1)), // Used to be 4
                                            driveMidRight(vex::motor(vex::PORT4, vex::ratio6_1)),
                                            driveBackRight(vex::motor(vex::PORT5, vex::ratio6_1, true)), // Used to be 15
                                             
                                            odomLift(vex::pneumatics(Brain.ThreeWirePort.G)), 

                                            leftDriveMotors(vex::motor_group(driveFrontLeft, driveBackLeft, driveMidLeft)),
                                            rightDriveMotors(vex::motor_group(driveFrontRight, driveBackRight, driveMidRight)), 

                                            startX(startX),
                                            startY(startY)
  {
    globalRef = this;
  };

  void init() override;
  void periodic() override;
  void updateTelemetry() override;
  void stop() override;

  void arcadeDrive(double speed, double rotation);

  void manualDriveForward(double speedMM, double centerAngle);
  void manualPercentageDrive(double decimal);
  void manualTurnCounterclockwise(double turnDeg);

  void manualDriveWithCurvature(double speedMM, double turnDeg); 

  void initializePosWithVertices(double x, double y); 
  void initializePosWithFaces(double x, double y); 

  void voltageDriveForward(double volts);
  void voltageTurnClockwise(double volts);

  void setCalibratingStructure(Alignment_Structure struc);
  void setStartingPos(double x, double y); 

  void scheduleSetpoint(double x, double y);

  static Location *getLocation(int index);
  PIDConstants getTurningPID();  
  
  PIDConstants getCorrectiveLinearPID(); 
  TrapezoidConstants getMotionConstants(); 
  PathMetadata getPathMetadata();  
  PathFrame getFrameData();

};

#endif