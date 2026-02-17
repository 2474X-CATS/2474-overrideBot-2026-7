#include "vex.h"
#include "architecture/robot.h"
#include <iostream>

#include "commands.h"
#include "gui/autonInit.h"

using namespace vex;

competition Competition;
Robot robot;

typedef struct
{
  string name;
  string desc;
  array<vector<CommandInterface *>, 2> autos;
} Routine;

AutonOption convertRoutineToOption(Routine routine, int index)
{
  AutonOption option;
  option.description = routine.desc;
  option.name = routine.name;
  option.hasLeftSide = !routine.autos.at(0).empty();
  option.hasRightSide = !routine.autos.at(1).empty();
  option.index = index;
  return option;
}

vector<AutonOption> getOptionVector(vector<Routine> routines)
{
  vector<AutonOption> res;
  for (int index = 0; index < routines.size(); index++)
  {
    res.push_back(convertRoutineToOption(routines.at(index), index));
  }
  return res;
}

void runTelemetry()
{
  robot.runTelemetryThread();
}

void awaitStartingSignal()
{
  LoadingScreen();
  Sprite::frameLoop();
}

int scheduleCallbacks()
{

  // Add a pause where you press a button to start
  //awaitStartingSignal(); 

  Competition.autonomous([]()
                         { robot.autonControl(); });
  Competition.drivercontrol([]()
                            { robot.driverControl(); });

  drawLogo(RobotState::getStateOf("is_team_color_blue"));
  return 0;
}

void testDrive()
{
  thread telemetryThread = thread(runTelemetry);
  robot.driverControl();
}

void testAuto(vector<CommandInterface *> auton, bool startingLeft)
{ 
  if (startingLeft){ 
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 2 + 430 - ROBOT_WIDTH_MM), 425); 
  } else { 
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 3 + 200), 425);
  }
  robot.setAutonomousCommand(auton);
  thread telemThread = thread(runTelemetry);
  robot.autonControl();
  robot.driverControl();
}

void configurateAutonomous(vector<Routine> routines)
{
  ColorPicker colorChooser = ColorPicker(135 + 115, 200);
  SidePicker sidePicker = SidePicker(235 + 115, 200);
  RoutineCatalog catalog = RoutineCatalog(10, 10, getOptionVector(routines));
  AutonDisplay(250, 80, &catalog, &sidePicker);
  ExitBlock(250, 10);
  Sprite::frameLoop();

  int sideIndex;
  int autonIndex = catalog.getCurrentAuto().index;

  if (sidePicker.getIsLeft())
  {
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 2 + 430 - ROBOT_WIDTH_MM), 425);
    sideIndex = 0;
  }
  else
  {
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 3 + 200), 425);
    sideIndex = 1;
  }

  robot.setAutonomousCommand(routines.at(autonIndex).autos.at(sideIndex));
  RobotState::manuallyModifyState("is_team_color_blue", colorChooser.getIsBlue());
}

void startCommandSkillsMatch(vector<CommandInterface *> commandGroup)
{
  robot.setAutonomousCommand(commandGroup);
  thread callBackTrigger = thread(scheduleCallbacks);
  robot.runTelemetryThread();
}

void startCommandCompetitiveMatch(vector<Routine> routines)
{
  configurateAutonomous(routines);
  thread callBackTrigger = thread(scheduleCallbacks);
  robot.runTelemetryThread();
}

void driveCommandMatch(vector<Routine> routines)
{
  configurateAutonomous(routines);
  thread telemThread = thread(runTelemetry);
  awaitStartingSignal();
  drawLogo(RobotState::getStateOf("is_team_color_blue"));
  robot.autonControl();
  robot.driverControl();
}

/*
-------------------------------------------------------------------------------------------------------------------------------------------------
     _              _______   _____    _____
    / \    |     |     |     |     |   |
   /___\   |     |     |     |     |   -----
  /     \  |_____|     |     |_____|   ____|
 --------------------------------------------------------------------------------------------------------------------------------------------------
*/

vector<CommandInterface *> closed_side_left()
{
  return {
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.95),
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM * 0.95, ROBOT_LENGTH_MM/2, true),
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM/2}, false, true),
      DeployMatchloader::getCommand(false),
      IntakeCubes::getCommand(400),
      ModifyRobotState::getCommand("is_drive_inverted", true),
     
      TurnToLocation(Zones::NAT_MID),
      DrivePath::getCommand({TILE_SIZE_MM*0.8}, false, false),  
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 2000), 
      DeployMatchloader::getCommand(true),  
      SlantedAlignWithX::getCommand(TILE_SIZE_MM + 25),  
      ModifyRobotState::getCommand("is_drive_inverted", false), 

      DrivePath::getCommand({270}, true, false), 
      CloseDistanceBetween(Zones::NAT_ML_LEFT, ROBOT_LENGTH_MM/2+250, 0, true),
      DriveForwardForTime::getCommand(0.2, 400, true), 
      IntakeCubes::getCommand(625),   
      ModifyRobotState::getCommand("is_drive_inverted", true), 
       
      TurnToLocation(Zones::NAT_HIGH_LEFT),
      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, ROBOT_LENGTH_MM / 2, 0, true),  
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.2, 750),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1250), 
      DisengageHighGoal::getCommand(0.2, 500), 

      DrivePath::getCommand({150}, true, false),
      DeployDescore::getCommand(true),   
      SlantedAlignWithX::getCommand(TILE_SIZE_MM - ((ROBOT_WIDTH_MM/2) + 125)), 
      DrivePath::getCommand({90}, true, false), 
      DriveForwardForTime::getCommand(0.3, 750, false),  
      WaitFor::getCommand(5000)
    };
}

vector<CommandInterface*> closed_side_right(){ 
  return {
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.95),
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 0.95, ROBOT_LENGTH_MM/2, true),
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM/2}, false, true),
      DeployMatchloader::getCommand(false),
      IntakeCubes::getCommand(400),
     
      TurnToLocation(Zones::NAT_LOW),
      DrivePath::getCommand({TILE_SIZE_MM*0.8}, false, false),  
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 2000), 
      DeployMatchloader::getCommand(true),   
     
      //DrivePath::getCommand({-sqrt(2) * 2 * TILE_SIZE_MM, 270}, false, false), 
      //DriveForwardForTime::getCommand(-0.2, 750, false),   
      //ModifyRobotState::getCommand("is_drive_inverted", true), 
      //WaitFor::getCommand(50),
      //DriveToSetpoint::getCommand(TILE_SIZE_MM * 5,TILE_SIZE_MM, -1, PathType::EUCLIDEAN, false),  
      SlantedAlignWithX::getCommand(TILE_SIZE_MM * 5),
      //ModifyRobotState::getCommand("is_drive_inverted", false),
      TurnToLocation(Zones::NAT_ML_RIGHT),
      CloseDistanceBetween(Zones::NAT_ML_RIGHT, ROBOT_LENGTH_MM/2+250, 0, true),
      DriveForwardForTime::getCommand(0.2, 400, true), 
      IntakeCubes::getCommand(625),   
      ModifyRobotState::getCommand("is_drive_inverted", true), 
      WaitFor::getCommand(50),
      TurnToLocation(Zones::NAT_HIGH_RIGHT),
      CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, ROBOT_LENGTH_MM / 2, 0, true),  
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, 0.2, 750),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1250), 
      DisengageHighGoal::getCommand(0.2, 500), 

      DrivePath::getCommand({30}, true, false),
      DeployDescore::getCommand(true),   
      SlantedAlignWithX::getCommand(TILE_SIZE_MM * 5 + ((ROBOT_WIDTH_MM/2) + 100)), 
      DrivePath::getCommand({90}, true, false), 
      DriveForwardForTime::getCommand(0.3, 750, false),  
      WaitFor::getCommand(5000)
    };
}

vector<CommandInterface *> renegade_right()
{
  return {  
      DeployDescore::getCommand(false),
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.95), // Get cubes near low
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 0.95, ROBOT_LENGTH_MM/2, true),  
      DeployMatchloader::getCommand(true),
      DriveForwardForTime::getCommand(0.2, 500, true),  
      IntakeCubes::getCommand(300),  
         
      ModifyRobotState::getCommand("is_drive_inverted", false),
      FlatAlignWithY::getCommand(TILE_SIZE_MM * 1), 
      FlatAlignWithX::getCommand(TILE_SIZE_MM * 5),  

      TurnToLocation(Zones::NAT_ML_RIGHT),  
      DriveForwardForTime::getCommand(0.2, 700, true),
      IntakeCubes::getCommand(500), 

      ModifyRobotState::getCommand("is_drive_inverted", true), 
      //DrivePath::getCommand({-1}, true, false), 
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, 0, 0, true), 
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, 0.3, 400), 
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1700),   
      DisengageHighGoal::getCommand(0.2, 700), 
 
      DrivePath::getCommand({30}, true, false),
      DeployDescore::getCommand(true),   
      SlantedAlignWithX::getCommand(TILE_SIZE_MM * 5 + ((ROBOT_WIDTH_MM/2) + 100)), 
      DrivePath::getCommand({90}, true, false), 
      DriveForwardForTime::getCommand(0.3, 750, false),  
      WaitFor::getCommand(5000)
      
    };
} 

vector<CommandInterface *> renegade_left()
{
  return {  
      DeployDescore::getCommand(false),
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.95), // Get cubes near low
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM * 0.95, ROBOT_LENGTH_MM/2, true),  
      DeployMatchloader::getCommand(true),
      DriveForwardForTime::getCommand(0.2, 500, true),  
      IntakeCubes::getCommand(300),  
         
      ModifyRobotState::getCommand("is_drive_inverted", false),
      FlatAlignWithY::getCommand(TILE_SIZE_MM * 1), 
      FlatAlignWithX::getCommand(TILE_SIZE_MM * 1),  

      TurnToLocation(Zones::NAT_ML_LEFT),  
      DriveForwardForTime::getCommand(0.2, 700, true),
      IntakeCubes::getCommand(500), 

      ModifyRobotState::getCommand("is_drive_inverted", true), 
      //DrivePath::getCommand({-1}, true, false), 
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, 0, 0, true), 
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.3, 400), 
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1700),   
      DisengageHighGoal::getCommand(0.2, 700), 
 
      DrivePath::getCommand({150}, true, false),
      DeployDescore::getCommand(true),   
      SlantedAlignWithX::getCommand(TILE_SIZE_MM - ((ROBOT_WIDTH_MM/2) + 125)), 
      DrivePath::getCommand({90}, true, false), 
      DriveForwardForTime::getCommand(0.3, 750, false),  
      WaitFor::getCommand(5000)
      
    };
}

vector<CommandInterface*> controlRushLeft(){ 
  return { 
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.95),
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM * 0.95, ROBOT_LENGTH_MM/2, true),
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM/2}, false, true),
      DeployMatchloader::getCommand(false),
      IntakeCubes::getCommand(400), 

      ModifyRobotState::getCommand("is_drive_inverted", true), 
      FlatAlignWithY::getCommand(TILE_SIZE_MM * 1.8), 
      DrivePath::getCommand({225}, true, false), //Tune the angle 1st 
      SlantedAlignWithX::getCommand(TILE_SIZE_MM), 
      DrivePath::getCommand({90}, true, false), 
      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, ROBOT_LENGTH_MM/2, 0, true), 
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.2, 500), 
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500),  

      DrivePath::getCommand({150}, true, false),
      DeployDescore::getCommand(true),   
      SlantedAlignWithX::getCommand(TILE_SIZE_MM - ((ROBOT_WIDTH_MM/2) + 125)), 
      DrivePath::getCommand({90}, true, false), 
      DriveForwardForTime::getCommand(0.3, 750, false),  
      WaitFor::getCommand(5000)

  };
}

vector<CommandInterface*> controlRushRight(){ 
  return { 
    TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.95),
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 0.95, ROBOT_LENGTH_MM/2, true),
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM/2}, false, true),
      DeployMatchloader::getCommand(false),
      IntakeCubes::getCommand(400), 

      ModifyRobotState::getCommand("is_drive_inverted", true), 
      FlatAlignWithY::getCommand(TILE_SIZE_MM * 1.8), 
      DrivePath::getCommand({315}, true, false), //Tune the angle 1st 
      SlantedAlignWithX::getCommand(TILE_SIZE_MM*5), 
      DrivePath::getCommand({90}, true, false), 
      CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, ROBOT_LENGTH_MM/2, 0, true), 
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, 0.2, 500), 
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500), 

      DrivePath::getCommand({30}, true, false),
      DeployDescore::getCommand(true),   
      SlantedAlignWithX::getCommand(TILE_SIZE_MM * 5 + ((ROBOT_WIDTH_MM/2) + 100)), 
      DrivePath::getCommand({90}, true, false), 
      DriveForwardForTime::getCommand(0.3, 750, false),  
      WaitFor::getCommand(5000)
  };
}

vector<CommandInterface*> centerCleanUpLeft(){ 
  return { 
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.95),
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM * 0.95, ROBOT_LENGTH_MM/2, true),
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM/2}, false, true),
      DeployMatchloader::getCommand(false),
      IntakeCubes::getCommand(400),
      ModifyRobotState::getCommand("is_drive_inverted", true), 

      TurnToLocation(Zones::NAT_MID),
      DrivePath::getCommand({TILE_SIZE_MM*0.8}, false, false),  
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 2000), 
      ModifyRobotState::getCommand("is_drive_inverted", false),  

      DrivePath::getCommand({TILE_SIZE_MM*0.8}, false, false),  
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM), 
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM, ROBOT_LENGTH_MM/2, false),  
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM/2+50}, false, true),  
      IntakeCubes::getCommand(400), 
      DeployMatchloader::getCommand(false), 
      
      TurnToLocation(Zones::NAT_LOW), 
      CloseDistanceBetween(Zones::NAT_LOW, ROBOT_LENGTH_MM/2, 0, false), 
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 1500),
  };
} 

vector<CommandInterface*> centerCleanUpRight(){ 
  return { 
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.95),
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 0.95, ROBOT_LENGTH_MM/2, true),
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM/2}, false, true),
      IntakeCubes::getCommand(400),
      DeployMatchloader::getCommand(false),

      TurnToLocation(Zones::NAT_LOW),
      DrivePath::getCommand({TILE_SIZE_MM*0.8}, false, false),  
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 1500), 
  

      DrivePath::getCommand({-TILE_SIZE_MM*0.8}, false, false),  
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM), 
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM, ROBOT_LENGTH_MM/2, false),  
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM/2+50}, false, true),  
      IntakeCubes::getCommand(400), 
      DeployMatchloader::getCommand(false), 
      ModifyRobotState::getCommand("is_drive_inverted", true), 

      TurnToLocation(Zones::NAT_MID), 
      CloseDistanceBetween(Zones::NAT_MID, ROBOT_LENGTH_MM/2, 0, false), 
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 2000),
  };
}

vector<CommandInterface *> auto_skills()
{
  return {   
      DriveForwardForTime::getCommand(0.2, 300, false), 
      ModifyRobotState::getCommand("is_drive_inverted", true), 
      WaitFor::getCommand(50),
      DriveToLocation(Zones::NAT_HIGH_RIGHT, TILE_SIZE_MM * 0.6, PathType::EUCLIDEAN, false),  
      TurnToLocation(Zones::NAT_HIGH_RIGHT), 
      CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, ROBOT_LENGTH_MM / 2 - 50, 0, true),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, 0.45, 850), 

      DeployMatchloader::getCommand(true), 
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500), 
      DisengageHighGoal::getCommand(0.2, 350), 

      ModifyRobotState::getCommand("is_drive_inverted", false), 
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::NAT_ML_RIGHT, 375, 0, false), 
      DriveForwardForTime::getCommand(0.25, 250, true), 
      IntakeCubes::getCommand(1250),  
      
      ModifyRobotState::getCommand("is_drive_inverted", true), 
      DriveForwardForTime::getCommand(0.25, 500, false),  
      DeployMatchloader::getCommand(false), 
  
      DriveToSetpoint::getCommand(TILE_SIZE_MM * 4.25, TILE_SIZE_MM * 1.6, -1, PathType::EUCLIDEAN, false),
      FlatAlignWithY::getCommand((TILE_SIZE_MM * 4) + (ROBOT_LENGTH_MM/3*2)), 
      FlatAlignWithX::getCommand(TILE_SIZE_MM * 5 - 40),   
      
      DrivePath::getCommand({270}, true, false),  
      Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_RIGHT, 0.25, 1000),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1750),   
      DeployMatchloader::getCommand(true),
      DisengageHighGoal::getCommand(0.2, 350), 

  
      ModifyRobotState::getCommand("is_drive_inverted", false), 
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::FAR_ML_RIGHT, 375, 0, false), 
      DriveForwardForTime::getCommand(0.25, 250, true),
      IntakeCubes::getCommand(1250),

      ModifyRobotState::getCommand("is_drive_inverted", true),  
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::FAR_HIGH_RIGHT, ROBOT_LENGTH_MM/2, 0, true), 
      Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_RIGHT, 0.25, 1000),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 2000), 
      DisengageHighGoal::getCommand(0.25, 750),  
      
      ModifyRobotState::getCommand("is_drive_inverted", true),
      FlatAlignWithX::getCommand(TILE_SIZE_MM+15),
      TurnToLocation(Zones::FAR_HIGH_LEFT),
      CloseDistanceBetween(Zones::FAR_HIGH_LEFT, ROBOT_LENGTH_MM/2, 0, false),  
      
      ModifyRobotState::getCommand("is_drive_inverted", true),
      Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_LEFT, 0.25, 1000), 
      

      ModifyRobotState::getCommand("is_drive_inverted", false), 
      WaitFor::getCommand(50), 
      CloseDistanceBetween(Zones::FAR_ML_LEFT, 375, 0, false),
      DriveForwardForTime::getCommand(0.25, 250, true), 
      IntakeCubes::getCommand(1250),  

      
      ModifyRobotState::getCommand("is_drive_inverted", true),   
      DriveForwardForTime::getCommand(0.3, 500, false),  
      DrivePath::getCommand({135, (sqrt(2) / 2) * TILE_SIZE_MM, 90, TILE_SIZE_MM * 3, 0, TILE_SIZE_MM * 0.55, 270}, true, false),
      
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.25, 1000),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500),   
      DisengageHighGoal::getCommand(0.2, 500), 

      ModifyRobotState::getCommand("is_drive_inverted", false),  
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::NAT_ML_LEFT, 375, 0, false), 
      DriveForwardForTime::getCommand(0.25, 250, true), 
      IntakeCubes::getCommand(1500), 

      ModifyRobotState::getCommand("is_drive_inverted", true), 
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, ROBOT_LENGTH_MM/2, 0, false),  
      
      ModifyRobotState::getCommand("is_drive_inverted", true),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.25, 1000),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500), 
      DisengageHighGoal::getCommand(0.2, 750),
      DeployMatchloader::getCommand(false), 

      ModifyRobotState::getCommand("is_drive_inverted", false),  
      DriveToSetpoint::getCommand(TILE_SIZE_MM * 2, TILE_SIZE_MM * 0.35, -1, PathType::EUCLIDEAN, false),
      DrivePath::getCommand({335}, true, false), 
      DrivePath::getCommand({750},false, true)  
      
  };
}

vector<CommandInterface*> empty(){ 
  return {  
    ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 300)
  };
}

vector<CommandInterface*> driveForward(){ 
  return { 
     DriveForwardForTime::getCommand(0.2, 500, false)
  };
}


//-------------------------------------------------------------------------------------------------------------------------------------------------

int main()
{

  vexcodeInit();

  //--------------------DONT MODIFY (MOSTLY)-----------------

  Drivebase drive = Drivebase(0,0); 
  Intake intake;
  Indexer indexer;
  Matchloader matchloader;
  Hooks hooks;

  robot.initialize();

  RobotState::manuallyModifyState("color_sensitive", false); // <- We don't have color-sort
  RobotState::manuallyModifyState("is_counterclockwise", false);

  //-------------------ROUTINE CREATION-------------------
  
  
  vector<Routine> routines;

  routines.push_back(
      (Routine){
          "Standard Play",
          "If on the left side: [4 blocks in mid - matchload 3 cubes - 3 blocks in long]. If right: [matchload 3 cubes - 7 blocks in long]",
          {
           closed_side_left(),
           renegade_right()
          } 
      } 
  );  

  routines.push_back( 
    (Routine){ 
      "7 Block High", 
      "Symmetrical: Gets middle cubes and matchloaded cubes and scores on the high goal", 
      { 
        renegade_left(), 
        renegade_right(), 
      }
    }
  );  

  routines.push_back( 
     (Routine){ 
        "2/3 AWP", 
        "Symmetrical: Grab near cubes then score on the center goal: (mid : left, low right) then score matchloaded cubes on high", 
        { 
          closed_side_left(), 
          closed_side_right()
        }
     }
  ); 

  routines.push_back( 
    (Routine){ 
      "Control Rush", 
      "Symmetrical: Score 3 blocks on the high goal, push to control then camp. [USE WISELY]", 
      { 
        controlRushLeft(), 
        controlRushRight(), 
      }
    }
  ); 

  routines.push_back( 
    (Routine){ 
        "Center Goal Clean", 
        "Score a combined 7 blocks on center goals - order depends on side [left mid, right low]", 
        { 
          centerCleanUpLeft(), 
          centerCleanUpRight()
        }
    }
  );


  routines.push_back(  
    (Routine){ 
      "Getting Carried", 
      "Our alliance has a FOR SURE AWP: (left : nothing), (right : forward a smidge)", 
      { 
        empty(), 
        driveForward()
      } 
    }
  );
  

  //-----------------------PROTOCOL------------------------


 //startCommandCompetitiveMatch(routines);    
 
 testAuto( 
   { 
    DrivePath::getCommand({135}, true, false), 
    ModifyRobotState::getCommand("is_drive_inverted", true), 
    WaitFor::getCommand(50),
    SlantedAlignWithX::getCommand(TILE_SIZE_MM * 4.5)
   },
   false
 );   

 /*
 
 testAuto( 
  { 
     DrivePath::getCommand({0}, true, false)
  }, 
  false
 ); 
 */
 /*
 Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 3 + 200), 425); 
 
 startCommandSkillsMatch( 
    auto_skills()
 );  
 */
 //startCommandCompetitiveMatch(routines);
 
 
 

}
