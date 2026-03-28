#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#include "subsystems/drivebase.h"
#include "subsystems/intake.h"
#include "subsystems/indexer.h"
#include "subsystems/matchloader.h"
#include "subsystems/hooks.h"
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
  NAT_ML_RIGHT = 1,
  NAT_HIGH_LEFT = 2,
  NAT_HIGH_RIGHT = 3,
  NAT_MID = 4,
  NAT_LOW = 5,
  NAT_PARK = 6,
  FAR_ML_RIGHT = 7,
  FAR_ML_LEFT = 8,
  FAR_HIGH_LEFT = 9,
  FAR_HIGH_RIGHT = 10,
  FAR_MID = 11,
  FAR_LOW = 12,
  FAR_PARK = 13
} Zones;

typedef enum
{
  EUCLIDEAN = 1,
  MANHATTAN_XY,
  MANHATTAN_YX
} PathType;

//---------------------------------------
// A COMBINATION OF DRIVING FORWARD AND TURNING COMMANDS

class DrivePath : public Command<Drivebase, Intake>
{

protected:
  Drivebase &drivebaseRef;
  Intake &intakeRef;

  vector<double> setpoints;
  bool turningFirst;
  bool intaking;

  pidcontroller *turnPID = nullptr;
  TrapezoidalMotionProfile *drivingProfile = nullptr;

  bool initialized = false;

  // bool isCounterClockwise;
  bool isGoingForward;
  double startingPoint[2];
  double lastPoint[2];

  double referenceAngle = 0;

  int operationsIndex = 0;
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

  void drivePeriodic();
  void turnPeriodic();

  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;
  virtual string repr() override;

public:
  // static bool isCounterClockwise;

  static CommandInterface *getCommand(vector<double> setpoints, bool turningFirst, bool intaking)
  {
    return new DrivePath(*Drivebase::globalRef, *Intake::globalRef, setpoints, turningFirst, intaking);
  }

  DrivePath(Drivebase &drive, Intake &intake, vector<double> setpoints, bool turningFirst, bool intaking) : Command<Drivebase, Intake>(drive, intake),
                                                                                                            drivebaseRef(drive),
                                                                                                            intakeRef(intake),
                                                                                                            setpoints(setpoints),
                                                                                                            turningFirst(turningFirst),
                                                                                                            intaking(intaking),
                                                                                                            numOfOperations(setpoints.size() - 1) {};
};

//---------------------------------------
// PURSUE A CERTAIN (X,Y) POSITION USING ONE OF THREE PATH TYPES

class DriveToSetpoint : protected DrivePath
{
protected:
  double setpointX;
  double setpointY;
  double endingAngle;
  PathType pathType;

public:
  static CommandInterface *getCommand(double setpointX, double setpointY, double endingAngle, PathType pathType, bool intaking)
  {
    return new DriveToSetpoint(*Drivebase::globalRef, *Intake::globalRef, setpointX, setpointY, endingAngle, pathType, intaking);
  }

  DriveToSetpoint(Drivebase &drive, Intake &intake, double setpointX, double setpointY, double endingAngle, PathType pathType, bool intaking) : DrivePath(drive, intake, {}, true, intaking),
                                                                                                                                                setpointX(setpointX),
                                                                                                                                                setpointY(setpointY),
                                                                                                                                                endingAngle(endingAngle),
                                                                                                                                                pathType(pathType) {};

protected:
  void start() override;
  string repr() override;
};

//-------------------------------------------------------------------------------------------------------------

class FlatAlignWithX : protected DriveToSetpoint
{
public:
  static CommandInterface *getCommand(double setpointX)
  {
    return new FlatAlignWithX(*Drivebase::globalRef, *Intake::globalRef, setpointX);
  }

  FlatAlignWithX(Drivebase &drive, Intake &intake, double setpointX) : DriveToSetpoint(drive, intake, setpointX, 0, -1, PathType::MANHATTAN_XY, false) {};

  void start() override;
};

class FlatAlignWithY : protected DriveToSetpoint
{
public:
  static CommandInterface *getCommand(double setpointY)
  {
    return new FlatAlignWithY(*Drivebase::globalRef, *Intake::globalRef, setpointY);
  }

  FlatAlignWithY(Drivebase &drive, Intake &intake, double setpointY) : DriveToSetpoint(drive, intake, 0, setpointY, -1, PathType::MANHATTAN_YX, false) {}

  void start() override;
};

//-------------------------------------------------------------------------------------------------------------

class SlantedAlignWithY : protected DrivePath
{
private:
  double setpointY;

public:
  static CommandInterface *getCommand(double setpointY)
  {
    return new SlantedAlignWithY(*Drivebase::globalRef, *Intake::globalRef, setpointY);
  }

  SlantedAlignWithY(Drivebase &drive, Intake &intake, double setpointY) : DrivePath(drive, intake, {}, false, false),
                                                                          setpointY(setpointY) {}

  void start() override;
};

class SlantedAlignWithX : protected DrivePath
{
private:
  double setpointX;

public:
  static CommandInterface *getCommand(double setpointX)
  {
    return new SlantedAlignWithX(*Drivebase::globalRef, *Intake::globalRef, setpointX);
  }

  SlantedAlignWithX(Drivebase &drive, Intake &intake, double setpointX) : DrivePath(drive, intake, {}, false, false),
                                                                          setpointX(setpointX) {}

  void start() override;
};

//------------------------------------------------------------------------------------------------------------- 
class FollowCirclePath : Command<Drivebase, Intake> { 
  private:  

     vector<BiarcEnum> setpoints;   
     vector<CirclePath> segments; 

     int index = 0;  
     int nSegments = 0; 

     bool initialized = false;  
     bool intaking;


     Drivebase& drivebaseRef;  
     Intake& intakeRef;  

  public:    

     static CommandInterface* getCommand(vector<BiarcEnum> setpoints, bool intaking){ 
        return new FollowCirclePath(*Drivebase::globalRef, *Intake::globalRef, setpoints, intaking);
     } 

     FollowCirclePath(Drivebase& drivebase, Intake& intake, vector<BiarcEnum> setpoints, bool intaking);
     
     void start() override; 
     void periodic() override; 
     bool isOver() override; 
     void end() override;

}; 

class FollowSplinePath : Command<Drivebase> { 
   private:  

     Drivebase& drivebaseRef; 
     HomingPath* path = nullptr;  
     array<array<double,2>,2> waypoints; 

   public:    

     static CommandInterface* getCommand(array<array<double,2>,2> waypoints){ 
         return new FollowSplinePath(*Drivebase::globalRef, waypoints);
     }
     
     FollowSplinePath(Drivebase& drivebase, array<array<double,2>,2> waypoints) :  
     Command<Drivebase>(drivebase),   
     drivebaseRef(drivebase),
     waypoints(waypoints){}; 

     void start() override; 
     void periodic() override; 
     bool isOver() override; 
     void end() override;


};

//---------------------------------------
// TURN THE ROBOT SO THAT IT S FACING A CERTAIN (X,Y) POSITION

class TurnToSetpoint : DriveToSetpoint
{

public:
  static CommandInterface *getCommand(double setpointX, double setpointY)
  {
    return new TurnToSetpoint(*Drivebase::globalRef, *Intake::globalRef, setpointX, setpointY);
  }

  TurnToSetpoint(Drivebase &drive, Intake &intake, double setpointX, double setpointY) : DriveToSetpoint(drive, intake, setpointX, setpointY, -1, PathType::EUCLIDEAN, false) {};

protected:
  void start() override;
  string repr() override;
};

class CloseDistance : DriveToSetpoint
{

private:
  double offset;

public:
  static CommandInterface *getCommand(double setpointX, double setpointY, double intaking, double offset)
  {
    return new CloseDistance(*Drivebase::globalRef, *Intake::globalRef, setpointX, setpointY, intaking, offset);
  }

  CloseDistance(Drivebase &drive, Intake &intake, double setpointX, double setpointY, bool intaking, double offset) : DriveToSetpoint(drive, intake, setpointX, setpointY, -1, PathType::EUCLIDEAN, intaking),
                                                                                                                      offset(offset) {};

protected:
  void start() override;
};

//---------------------------------------
// DRIVES THE ROBOT FORWARD OR BACKWARD AT A CERTAIN SPEED PERCENTAGE

class DriveForwardForTime : Command<Drivebase, Intake>
{
private:
  Drivebase &drivebaseRef;
  Intake &intakeRef;

  bool intaking;

  double percentage;
  double startingTime;
  double timeDuration;

protected:
  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;
  string repr() override;

public:
  static CommandInterface *getCommand(double percentage, double timeDuration, bool intaking)
  {
    return new DriveForwardForTime(*Drivebase::globalRef, *Intake::globalRef, percentage, timeDuration, intaking);
  }

  DriveForwardForTime(Drivebase &drivebase, Intake &intake, double percentage, double timeDuration, bool intaking) : Command<Drivebase, Intake>(drivebase, intake),
                                                                                                                     intaking(intaking),
                                                                                                                     drivebaseRef(drivebase),
                                                                                                                     intakeRef(intake),
                                                                                                                     percentage(percentage),
                                                                                                                     timeDuration(timeDuration) {};

  ~DriveForwardForTime() override = default;
};

class Calibrate : Command<Drivebase>
{
private:
  Drivebase &drivebaseRef;
  double percentage;
  double durationMilliseconds;
  double startingTime;
  Alignment_Structure wall;

protected:
  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;

public:
  static CommandInterface *getCommand(Alignment_Structure wall, double percentageOutput, double durationMillis)
  {
    return new Calibrate(*Drivebase::globalRef, wall, percentageOutput, durationMillis);
  }

  Calibrate(Drivebase &drivebase, Alignment_Structure wall, double percentageOutput, double durationMillis) : Command<Drivebase>(drivebase),
                                                                                                              drivebaseRef(drivebase),
                                                                                                              percentage(percentageOutput),
                                                                                                              durationMilliseconds(durationMillis),
                                                                                                              wall(wall) {};

  ~Calibrate() override = default;
};

//---------------------------------------
// INTAKE CUBES FOR STORAGE FOR A CERTAIN AMOUNT OF TIME

class IntakeCubes : public Command<Intake>
{
private:
  Intake &intakeRef;

  double startingTime;
  double timeDuration;

protected:
  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;
  string repr() override;

public:
  static CommandInterface *getCommand(double timeDuration)
  {
    return new IntakeCubes(*Intake::globalRef, timeDuration);
  }

  IntakeCubes(Intake &intake, double timeDuration) : Command<Intake>(intake),
                                                     intakeRef(intake),
                                                     timeDuration(timeDuration) {};

  ~IntakeCubes() override = default;
};

// SCORE ON EITHER A HIGH, MID, OR LOW GOAL FOR A CERTAIN AMOUNT OF TIME 

class ScoreOnGoal : public Command<Intake, Indexer>
{
private:
  double timeDuration;
  int goal;

public:
  static CommandInterface *getCommand(int goal, double timeDuration)
  {
    return new ScoreOnGoal(*Intake::globalRef, *Indexer::globalRef, goal, timeDuration);
  };

  ScoreOnGoal(Intake &intake, Indexer &indexer, int goal, double timeDuration) : Command<Intake, Indexer>(intake, indexer),
                                                                                                             intakeRef(intake),
                                                                                                             indexerRef(indexer),
                                                                                                             timeDuration(timeDuration),
                                                                                                             goal(goal) {};

  ~ScoreOnGoal() override = default;

protected: 
  Intake &intakeRef;
  Indexer &indexerRef; 
  
  double startingTime; 
  
  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override; 
  string repr() override;  
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
  string repr() override;

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

class DeployDescore : Command<Hooks>
{
private:
  Hooks &hooksRef;
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
    return new DeployDescore(*Hooks::globalRef, out);
  }

  DeployDescore(Hooks &hooks, bool out) : Command<Hooks>(hooks),
                                          hooksRef(hooks),
                                          isOut(out) {};

  ~DeployDescore() override = default;
};

//---------------------------------------
// BLOCKS THE ROUTINE FROM RUNNING FOR A CERTAIN AMOUNT OF TIME

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
  string repr() override;
};

//---------------------------------------
// MODIFIES THE STATE OF THE ROBOT

class ModifyRobotState : Command<DummySystem>
{
public:
  string entryKey;
  bool value;

  static CommandInterface *getCommand(string key, bool val)
  {
    return new ModifyRobotState(key, val);
  }

  ModifyRobotState(string key, bool val) : Command<DummySystem>(GLOBAL_DUMMY),
                                           entryKey(key),
                                           value(val) {};

  ~ModifyRobotState() override = default;

protected:
  void start() override;
  void periodic() override;
  bool isOver() override;
  void end() override;
  string repr() override;
};

CommandInterface *DriveToLocation(int zoneIndex, double dist, PathType pathType, bool intaking);

CommandInterface *TurnToLocation(int zoneIndex);
CommandInterface *TurnToLocation(int zoneIndex, double dist);

CommandInterface *CloseDistanceBetween(int zoneIndex, double dist, double offset, bool intaking);

#endif