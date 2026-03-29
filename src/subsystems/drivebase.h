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

  // vex::rotation encoderLinear;
  // vex::rotation encoderAngular;

  vex::inertial driveGyro;

  vex::motor driveFrontLeft;
  // vex::motor driveMidLeft;
  vex::motor driveBackLeft;

  vex::motor driveFrontRight;
  // vex::motor driveMidRight;
  vex::motor driveBackRight;

  vex::motor_group leftDriveMotors;
  vex::motor_group rightDriveMotors;

  PIDConstants turnPID; 
  PIDConstants correctivePID;

  TrapezoidConstants trapConsts;

  double startX, startY;
  double linearSpeedFactor = 1;
  double angularSpeedFactor = 1;

  double lastTimestamp = 0;

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
                                                 (EntrySet){"Instantaneous_Speed", EntryType::DOUBLE}
                                                }),
                                            // encoderLinear(vex::rotation(vex::PORT9)),
                                            // encoderAngular(vex::rotation(vex::PORT10)),
                                            driveGyro(vex::inertial(vex::PORT20)),                  // Used to be port 16
                                            driveFrontLeft(vex::motor(vex::PORT10, vex::ratio6_1)), // Used to be 1
                                            // driveMidLeft(vex::motor(vex::PORT2, vex::ratio6_1, true)),
                                            driveBackLeft(vex::motor(vex::PORT9, vex::ratio6_1)),   // Used to be 3
                                            driveFrontRight(vex::motor(vex::PORT1, vex::ratio6_1)), // Used to be 4
                                            // driveMidRight(vex::motor(vex::PORT5, vex::ratio6_1, true)),
                                            driveBackRight(vex::motor(vex::PORT2, vex::ratio6_1)), // Used to be 15
                                            leftDriveMotors(vex::motor_group(driveFrontLeft, driveBackLeft)),
                                            rightDriveMotors(vex::motor_group(driveFrontRight, driveBackRight)),
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

  void voltageDriveForward(double volts);
  void voltageTurnClockwise(double volts);

  void setCalibratingStructure(Alignment_Structure struc);
  void setStartingPos(double x, double y);

  static Location *getLocation(int index);
  PIDConstants getTurningPID();
  TrapezoidConstants getMotionConstants(); 
  PathMetadata getPathMetadata(); 

};

#endif