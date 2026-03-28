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

  vex::rotation encoderLinear;
  //vex::rotation encoderAngular;

  vex::inertial driveGyro;

  vex::motor driveFrontLeft;
  vex::motor driveMidLeft;
  vex::motor driveBackLeft;

  vex::motor driveFrontRight;
  vex::motor driveMidRight;
  vex::motor driveBackRight;

  vex::motor_group leftDriveMotors;
  vex::motor_group rightDriveMotors;

  PIDConstants turnPID; 
  PIDConstants correctivePID;

  TrapezoidConstants trapConsts; 

  double linearSpeedFactor = 1;
  double angularSpeedFactor = 1;

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

  Drivebase(double startX, double startY);

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