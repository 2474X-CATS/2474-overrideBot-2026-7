#ifndef __COMMANDS_H__ 
#define __COMMANDS_H__ 

#include "subsystems/drivebase.h"  
#include "subsystems/intake.h"  
#include "subsystems/hood.h"  
#include "subsystems/matchloader.h"  
#include "subsystems/indexer.h" 
#include "subsystems/hopper.h"  
#include "architecture/command.h"  


typedef enum { 
   HIGH_GOAL = 1, 
   MID_GOAL = 2, 
   LOW_GOAL = 3
} Goal_Pos; 

typedef enum { 
  NAT_ML_LEFT = 0, 
  NAT_ML_RIGHT, 
  NAT_HIGH_LEFT, 
  NAT_HIGH_RIGHT, 
  NAT_MID, 
  NAT_LOW, 
  NEAR_PARK, 
  FOR_ML_RIGHT, 
  FOR_ML_LEFT, 
  FOR_HIGH_LEFT, 
  FOR_HIGH_RIGHT, 
  FOR_MID, 
  FOR_LOW, 
  FAR_PARK
} Zones; 

typedef enum { 
   EUCLIDEAN, 
   MANHATTAN_XY, 
   MANHATTAN_YX
} PathType;


class DrivePath : public Command<Drivebase> { 
  
  protected: 
      Drivebase& drivebaseRef; 
      vector<double> setpoints;   
      bool turningFirst;

      pidcontroller* turnPID = nullptr; 
      pidcontroller* drivePID = nullptr;  
      
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

     static CommandInterface *getCommand(vector<double> setpoints, bool turningFirst)
     {
         return new DrivePath(*Drivebase::globalRef, setpoints, turningFirst);
     }   
   /*
     static CommandInterface* getCommand(int locationIndex, double endpointDist){ 
         return new DrivePath(*Drivebase::globalRef, *Drivebase::globalRef->getLocation(locationIndex)->getEuclideanAlignmentPath(endpointDist), true); 
     } 

     static CommandInterface* getCommand(int locationIndex, double endpointDist, bool xFirst){ 
         return new DrivePath(*Drivebase::globalRef, *Drivebase::globalRef->getLocation(locationIndex)->getTaxicabAlignmentPath(endpointDist, xFirst), true);
     }
   */
     DrivePath(Drivebase& drive, vector<double> setpoints, bool turningFirst) :  
     Command<Drivebase>(drive), 
     drivebaseRef(drive), 
     setpoints(setpoints),  
     turningFirst(turningFirst),
     initialized(false), 
     operationsIndex(0),
     numOfOperations(setpoints.size() - 1)
     {}; 
}; 

class DriveToSetpoint : DrivePath {  
   private:  
     double setpointX; 
     double setpointY;  
     double endingAngle;  
     PathType pathType;

   public:   

     static CommandInterface *getCommand(int locationIndex, double distFrom, PathType pathType)
     { 
         Location* location = Drivebase::globalRef->getLocation(locationIndex);   
         array<double, 2> setpoint = location->getProjectedSetpoint(distFrom);
         return new DriveToSetpoint(*Drivebase::globalRef, setpoint.at(0), setpoint.at(1), location->getPerfectEntranceAngle(), pathType); 
     }  

     static CommandInterface *getCommand(double setpointX, double setpointY, PathType pathType)
     { 
         return new DriveToSetpoint(*Drivebase::globalRef, setpointX, setpointY, 90, pathType); 
     } 

     DriveToSetpoint(Drivebase& drive, double setpointX, double setpointY, double endingAngle, PathType pathType) :  
     DrivePath(drive, {}, true),  
     setpointX(setpointX), 
     setpointY(setpointY), 
     endingAngle(endingAngle), 
     pathType(pathType) 
     {}; 

   protected: 
     void start() override; 

}; 


class DriveForwardForTime : Command<Drivebase> { 
    private:  
      Drivebase& drivebaseRef;
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

      DriveForwardForTime(Drivebase& drivebase, double percentage, double timeDuration) :  
      Command<Drivebase>(drivebase), 
      drivebaseRef(drivebase),
      percentage(percentage), 
      timeDuration(timeDuration){};   

      ~DriveForwardForTime() override = default;

}; 

/*
class TurnToHeading : public Command<Drivebase> { 
    private:  
      Drivebase &driveRef; 
      pidcontroller* control = nullptr; 

      bool isClockwise; 
      double getAngluarDifference(); 

      double angleSetpoint;

    public:
      static CommandInterface *getCommand(double heading)
      {
         return new TurnToHeading(*Drivebase::globalRef, heading);
      }     

      TurnToHeading(Drivebase &drive, double degreeHeading) : 
      Command<Drivebase>(drive), 
      driveRef(drive), 
      control(new pidcontroller(drive.getTurningPID(), 0)),
      angleSetpoint(degreeHeading){};  

      ~TurnToHeading() override = default;

    protected:   
      void start() override; 
      void periodic() override;  
      bool isOver() override; 
      void end() override;
};

*/

class IntakeToHopper : public Command<Intake, Indexer, Hood> { 
    private:   

      Intake& intakeRef;  
      Indexer& indexerRef;  
      Hood& hoodRef; 
      
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

      IntakeToHopper(Intake& intake, Indexer& indexer, Hood& hood, double timeDuration) : 
      Command<Intake, Indexer, Hood>(intake, indexer, hood), 
      intakeRef(intake), 
      indexerRef(indexer), 
      hoodRef(hood), 
      timeDuration(timeDuration){};   

      ~IntakeToHopper() override = default;

};

class ScoreOnGoal : public Command<Intake, Indexer, Hood, Hopper> { 
    private:   

      Intake& intakeRef;  
      Indexer& indexerRef;  
      Hood& hoodRef;  
      Hopper& hopperRef;
      
      double startingTime;
      double timeDuration;   
      int goal;
    
    public: 
      static CommandInterface *getCommand(int goal, double timeDuration)
      {
         return new ScoreOnGoal(*Intake::globalRef, *Indexer::globalRef, *Hood::globalRef, *Hopper::globalRef, goal, timeDuration);
      };

      ScoreOnGoal(Intake& intake, Indexer& indexer, Hood& hood, Hopper& hopper, int goal, double timeDuration) : 
      Command<Intake, Indexer, Hood, Hopper>(intake, indexer, hood, hopper), 
      intakeRef(intake), 
      indexerRef(indexer),  
      hoodRef(hood), 
      hopperRef(hopper),  
      timeDuration(timeDuration), 
      goal(goal){};   

      ~ScoreOnGoal() override = default; 

    protected: 
      void start() override;
      void periodic() override;
      bool isOver() override;
      void end() override; 

}; 


class DeployMatchloader : Command<Matchloader> { 
    private:  
      Matchloader& matchLoaderRef;
      bool isOut;
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

      DeployMatchloader(Matchloader& matchloader, bool out) :  
      Command<Matchloader>(matchloader), 
      matchLoaderRef(matchloader), 
      isOut(out){};   

      ~DeployMatchloader() override = default;

}; 

class WaitFor : Command<DummySystem> {  
     private:   

       double timeDuration;
       double startTime; 

     public: 
      static CommandInterface *getCommand(double timeDuration)
      {
         return new WaitFor(timeDuration);
      }  

      WaitFor(double timeDuration) :  
      Command<DummySystem>(GLOBAL_DUMMY), 
      timeDuration(timeDuration){};   

      ~WaitFor() override = default; 

     protected:
      void start() override;
      void periodic() override;
      bool isOver() override;
      void end() override; 

};

//CommandInterface* driveForwardByTiles(double tiles); 
//CommandInterface* turnToAngle(double goalHeading); 
CommandInterface* alignWithLocation(int locationIndex, double projectedDist, PathType pathType); 
CommandInterface* driveToPoint(double setX, double setY, PathType pathType);
CommandInterface* scoreOnGoal(Goal_Pos position, double timeDuration);
CommandInterface* intakeCubes(double timeDuration);
CommandInterface* holdFor(double timeDuration);
CommandInterface* extend(); 
CommandInterface* retract(); 
//CommandInterface* driveAndIntakeForTiles(double tiles);  
CommandInterface* ramForwardFor(double percentage, double timeDuration); 


/*
string makePreciseDrive(bool intaking);
string makePreciseTurn();
string makeScoreCubes();
string makeIntakeCubes();
string makeRamForward();
*/



#endif 