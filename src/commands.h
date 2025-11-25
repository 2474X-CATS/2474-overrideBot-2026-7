#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#include "subsystems/drivebase.h"
#include "subsystems/intake.h"
#include "subsystems/hood.h"
#include "subsystems/matchloader.h"
#include "subsystems/indexer.h"
#include "subsystems/hopper.h"
#include "architecture/command.h"

typedef enum
{
  HIGH_GOAL = 1,
  MID_GOAL = 2,
  LOW_GOAL = 3
} Goal_Pos;

typedef enum
{
  NAT_ML_LEFT = 0,
  NAT_ML_RIGHT,
  NAT_HIGH_LEFT,
  NAT_HIGH_RIGHT,
  NAT_MID,
  NAT_LOW,
  NAT_PARK,
  FAR_ML_RIGHT,
  FAR_ML_LEFT,
  FAR_HIGH_LEFT,
  FAR_HIGH_RIGHT,
  FAR_MID,
  FAR_LOW,
  FAR_PARK
} Zones;

typedef enum
{
  EUCLIDEAN,
  MANHATTAN_XY,
  MANHATTAN_YX
} PathType;

class DrivePath : public Command<Drivebase, Intake, Indexer, Hood>
{

protected:
  Drivebase &drivebaseRef;

  Intake &intakeRef;
  Indexer &indexerRef;
  Hood &hoodRef;

  bool intaking;

  vector<double> setpoints;
  bool turningFirst;

  pidcontroller *turnPID = nullptr;
  pidcontroller *drivePID = nullptr;

  bool initialized;

  bool isCounterClockwise;

  bool isGoingForward;
  double startingPoint[2];

  int operationsIndex;
  int numOfOperations;

  void initializeDrive();
  void initializeTurn();

  bool isDriveOver();
  bool isTurnOver();

  void drive();
  void turn();

  bool isTurning();
  bool isDriving();

  double getAngularError();
  double getDrivingError();

  void drivePeriodic();
  void turnPeriodic();

  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;

public:
  static CommandInterface *getCommand(vector<double> setpoints, bool turningFirst, bool intaking)
  {
    return new DrivePath(*Drivebase::globalRef, *Intake::globalRef, *Indexer::globalRef, *Hood::globalRef, setpoints, turningFirst, intaking);
  }

  DrivePath(Drivebase &drive, Intake &intake, Indexer &indexer, Hood &hood, vector<double> setpoints, bool turningFirst, bool intaking) : Command<Drivebase, Intake, Indexer, Hood>(drive, intake, indexer, hood),
                                                                                                                                          drivebaseRef(drive),
                                                                                                                                          intakeRef(intake),
                                                                                                                                          indexerRef(indexer),
                                                                                                                                          hoodRef(hood),
                                                                                                                                          intaking(intaking),
                                                                                                                                          setpoints(setpoints),
                                                                                                                                          turningFirst(turningFirst),
                                                                                                                                          initialized(false),
                                                                                                                                          operationsIndex(0),
                                                                                                                                          numOfOperations(setpoints.size() - 1) {};
};

class DriveToSetpoint : DrivePath
{
private:
  double setpointX;
  double setpointY;
  double endingAngle;
  PathType pathType;

public:
  static CommandInterface *getCommand(double setpointX, double setpointY, double endingAngle, PathType pathType, bool intaking)
  {
    return new DriveToSetpoint(*Drivebase::globalRef, *Intake::globalRef, *Indexer::globalRef, *Hood::globalRef, setpointX, setpointY, endingAngle, pathType, intaking);
  }

  DriveToSetpoint(Drivebase &drive, Intake &intake, Indexer &indexer, Hood &hood, double setpointX, double setpointY, double endingAngle, PathType pathType, bool intaking) : DrivePath(drive, intake, indexer, hood, {}, true, intaking),
                                                                                                                                                                              setpointX(setpointX),
                                                                                                                                                                              setpointY(setpointY),
                                                                                                                                                                              endingAngle(endingAngle),
                                                                                                                                                                              pathType(pathType) {};

protected:
  void start() override;
};

class DriveForwardForTime : Command<Drivebase>
{
private:
  Drivebase &drivebaseRef;
  double percentage;

  double startingTime;

  double timeDuration;

protected:
  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;

public:
  static CommandInterface *getCommand(double percentage, double timeDuration)
  {
    return new DriveForwardForTime(*Drivebase::globalRef, percentage, timeDuration);
  }

  DriveForwardForTime(Drivebase &drivebase, double percentage, double timeDuration) : Command<Drivebase>(drivebase),
                                                                                      drivebaseRef(drivebase),
                                                                                      percentage(percentage),
                                                                                      timeDuration(timeDuration) {};

  ~DriveForwardForTime() override = default;
};

class IntakeToHopper : public Command<Intake, Indexer, Hood>
{
private:
  Intake &intakeRef;
  Indexer &indexerRef;
  Hood &hoodRef;

  double startingTime;
  double timeDuration;

protected:
  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;

public:
  static CommandInterface *getCommand(double timeDuration)
  {
    return new IntakeToHopper(*Intake::globalRef, *Indexer::globalRef, *Hood::globalRef, timeDuration);
  }

  IntakeToHopper(Intake &intake, Indexer &indexer, Hood &hood, double timeDuration) : Command<Intake, Indexer, Hood>(intake, indexer, hood),
                                                                                      intakeRef(intake),
                                                                                      indexerRef(indexer),
                                                                                      hoodRef(hood),
                                                                                      timeDuration(timeDuration) {};

  ~IntakeToHopper() override = default;
};

class ScoreOnGoal : public Command<Intake, Indexer, Hood, Hopper>
{
private:
  Intake &intakeRef;
  Indexer &indexerRef;
  Hood &hoodRef;
  Hopper &hopperRef;

  double startingTime;
  double timeDuration;
  int goal;

public:
  static CommandInterface *getCommand(int goal, double timeDuration)
  {
    return new ScoreOnGoal(*Intake::globalRef, *Indexer::globalRef, *Hood::globalRef, *Hopper::globalRef, goal, timeDuration);
  };

  ScoreOnGoal(Intake &intake, Indexer &indexer, Hood &hood, Hopper &hopper, int goal, double timeDuration) : Command<Intake, Indexer, Hood, Hopper>(intake, indexer, hood, hopper),
                                                                                                             intakeRef(intake),
                                                                                                             indexerRef(indexer),
                                                                                                             hoodRef(hood),
                                                                                                             hopperRef(hopper),
                                                                                                             timeDuration(timeDuration),
                                                                                                             goal(goal) {};

  ~ScoreOnGoal() override = default;

protected:
  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;
};

class DeployMatchloader : Command<Matchloader>
{
private:
  Matchloader &matchLoaderRef;
  bool isOut;
  bool ran = false;

protected:
  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;

public:
  static CommandInterface *getCommand(bool out)
  {
    return new DeployMatchloader(*Matchloader::globalRef, out);
  }

  DeployMatchloader(Matchloader &matchloader, bool out) : Command<Matchloader>(matchloader),
                                                          matchLoaderRef(matchloader),
                                                          isOut(out) {};

  ~DeployMatchloader() override = default;
};

class WaitFor : Command<DummySystem>
{
private:
  double timeDuration;
  double startTime;

public:
  static CommandInterface *getCommand(double timeDuration)
  {
    return new WaitFor(timeDuration);
  }

  WaitFor(double timeDuration) : Command<DummySystem>(GLOBAL_DUMMY),
                                 timeDuration(timeDuration) {};

  ~WaitFor() override = default;

protected:
  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;
};

CommandInterface *DriveLinear(double distance, bool intaking);
CommandInterface *TurnToHeading(double angle);
CommandInterface *DriveToPoint(double setpointX, double setpointY, PathType pathType, bool intaking);
CommandInterface *AlignWithLocation(int locationIndex, double distance, PathType pathType, bool intaking);
CommandInterface *Score(Goal_Pos pos, double duration);
CommandInterface *IntakeCubes(double duration);
CommandInterface *EnableMatchloader(bool enabled);
CommandInterface *Wait(double duration); 

CommandInterface* RamForward(double percentage, double duration);

#endif