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
  awaitStartingSignal();

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
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM),
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM, 250, true),
      DeployMatchloader::getCommand(true),
      DriveForwardForTime::getCommand(0.25, 600, true),
      DeployMatchloader::getCommand(false),
      IntakeCubes::getCommand(500),
      ModifyRobotState::getCommand("is_drive_inverted", true),
     
      TurnToLocation(Zones::NAT_MID),
      DrivePath::getCommand({TILE_SIZE_MM * 0.8}, false, false), 
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 2000), 
      DeployMatchloader::getCommand(true),
      ModifyRobotState::getCommand("is_drive_inverted", false),
     
      DrivePath::getCommand({1100 + ROBOT_LENGTH_MM / 2, 270}, false, false), //Was 1100 
      DriveForwardForTime::getCommand(0.3, 500, true), 
      IntakeCubes::getCommand(500), 
      ModifyRobotState::getCommand("is_drive_inverted", true), 
      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, ROBOT_LENGTH_MM / 2 - 100, 0, true),  
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.2, 700),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500), 
      DisengageHighGoal::getCommand(0.2, 500) 
    
    };
}

vector<CommandInterface *> closed_side_right()
{
  return {  
      DeployDescore::getCommand(false),
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM), // Get cubes near low
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM, ROBOT_LENGTH_MM/2, true),  
      DeployMatchloader::getCommand(true),
      DriveForwardForTime::getCommand(0.2, 500, true),  
      IntakeCubes::getCommand(200),  
         
      ModifyRobotState::getCommand("is_drive_inverted", false),
      FlatAlignWithY::getCommand(TILE_SIZE_MM * 1), 
      FlatAlignWithX::getCommand(TILE_SIZE_MM * 5),  

      TurnToLocation(Zones::NAT_ML_RIGHT),  
      DriveForwardForTime::getCommand(0.3, 400, true),
      IntakeCubes::getCommand(300), 

      ModifyRobotState::getCommand("is_drive_inverted", true), 
      TurnToLocation(Zones::NAT_HIGH_RIGHT),
      CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, 0, 0, true), 
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, 0.3, 400), 
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500),   
      DisengageHighGoal::getCommand(0.2, 500),
      FlatAlignWithY::getCommand(TILE_SIZE_MM + (ROBOT_LENGTH_MM / 2)), 
      DrivePath::getCommand({30}, true, false),
      DeployDescore::getCommand(true),   
      SlantedAlignWithX::getCommand(TILE_SIZE_MM * 5 + (ROBOT_WIDTH_MM/2)) 

    };
}

vector<CommandInterface *> auto_skills()
{
  return {

      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM), 
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM, ROBOT_LENGTH_MM/2, true),  
      DeployMatchloader::getCommand(true),
      DriveForwardForTime::getCommand(0.2, 500, true),  
      IntakeCubes::getCommand(200), 
     
      ModifyRobotState::getCommand("is_drive_inverted", true),
      FlatAlignWithY::getCommand((TILE_SIZE_MM * 2) - (ROBOT_LENGTH_MM * 1.125)), 
      FlatAlignWithX::getCommand(TILE_SIZE_MM * 5),
      TurnToLocation(Zones::NAT_HIGH_RIGHT),
      CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, ROBOT_LENGTH_MM / 2, 0, false),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, 0.35, 1000),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500), 
      DisengageHighGoal::getCommand(0.2, 500), 

      ModifyRobotState::getCommand("is_drive_inverted", false),
      DriveToLocation(Zones::NAT_ML_RIGHT, 475, PathType::EUCLIDEAN, false), // Matchload cubes from 4
      DriveForwardForTime::getCommand(0.2, 750, true),
      IntakeCubes::getCommand(2000),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      FlatAlignWithY::getCommand(TILE_SIZE_MM * 1.15),  
      DeployMatchloader::getCommand(false),
      FlatAlignWithX::getCommand(TILE_SIZE_MM * 4.25), 
      FlatAlignWithY::getCommand((TILE_SIZE_MM * 4) + (ROBOT_LENGTH_MM)), 
      FlatAlignWithX::getCommand(TILE_SIZE_MM * 5), 
      TurnToLocation(Zones::FAR_HIGH_RIGHT), 
      CloseDistanceBetween(Zones::FAR_HIGH_RIGHT, (ROBOT_LENGTH_MM/2), 0, false),
      Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_RIGHT, 0.45, 100),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500),  
      DisengageHighGoal::getCommand(0.2, 350),
      DeployMatchloader::getCommand(true),
      
      ModifyRobotState::getCommand("is_drive_inverted", false),
      CloseDistanceBetween(Zones::FAR_ML_RIGHT, 475, 0, false), 
      DriveForwardForTime::getCommand(0.2, 750, true),
      IntakeCubes::getCommand(1500), 

      ModifyRobotState::getCommand("is_drive_inverted", true), 
      CloseDistanceBetween(Zones::FAR_HIGH_RIGHT, ROBOT_LENGTH_MM/2, 0, false), 
      Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_RIGHT, 0.25, 1000),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500), 
      DisengageHighGoal::getCommand(0.2, 500),
      
      FlatAlignWithY::getCommand(TILE_SIZE_MM * 4.5), 
      FlatAlignWithX::getCommand(TILE_SIZE_MM),
      TurnToLocation(Zones::FAR_HIGH_LEFT),
      CloseDistanceBetween(Zones::FAR_HIGH_LEFT, ROBOT_LENGTH_MM/2, 0, false), 
      Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_LEFT, 0.3, 1250),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      CloseDistanceBetween(Zones::FAR_ML_LEFT, 475, 0, false),
      DriveForwardForTime::getCommand(0.2, 750, true),
      IntakeCubes::getCommand(1500),   

      ModifyRobotState::getCommand("is_drive_inverted", true),  
      FlatAlignWithY::getCommand(TILE_SIZE_MM * 5), 
      FlatAlignWithX::getCommand(TILE_SIZE_MM * 1.5), 
      FlatAlignWithY::getCommand(TILE_SIZE_MM + (ROBOT_LENGTH_MM/2)), 
      FlatAlignWithX::getCommand(TILE_SIZE_MM), 
      TurnToLocation(Zones::NAT_HIGH_LEFT), 
      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, ROBOT_LENGTH_MM/2, 0, false),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.25, 500),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500),   
      DisengageHighGoal::getCommand(0.2, 500),
 
      ModifyRobotState::getCommand("is_drive_inverted", false), 
      CloseDistanceBetween(Zones::NAT_ML_LEFT, 475, 0, false), 
      DriveForwardForTime::getCommand(0.2, 750, true), 
      IntakeCubes::getCommand(1500), 

      ModifyRobotState::getCommand("is_drive_inverted", true),
      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, ROBOT_LENGTH_MM/2, 0, false), 
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.25, 1000),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500), 
      DisengageHighGoal::getCommand(0.2, 500),
      DeployMatchloader::getCommand(false), 

      ModifyRobotState::getCommand("is_drive_inverted", false),
      DriveToLocation(Zones::NAT_PARK, TILE_SIZE_MM, PathType::MANHATTAN_YX, false),  
      FlatAlignWithY::getCommand(TILE_SIZE_MM * 1.33),  
      FlatAlignWithX::getCommand(TILE_SIZE_MM * 3), 
      TurnToLocation(Zones::NAT_PARK),

      DriveForwardForTime::getCommand(-0.2, 500, false),
      DriveForwardForTime::getCommand(1, 3000, true), 
      
  };
}

vector<CommandInterface*> empty(){ 
  return { 
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
          "Standard 1/2 AWP",
          "If on the left side: [4 blocks in mid - matchload 3 cubes - 3 blocks in long]. If right: [matchload 3 cubes - 7 blocks in long]",
          {
           closed_side_left(),
           closed_side_right()
          } 
      } 
  );  
  

  //-----------------------PROTOCOL------------------------


 testAuto( 
  { 
     DrivePath::getCommand({135}, true, false), 
     SlantedAlignWithX::getCommand(TILE_SIZE_MM * 2)
  }, 
  true);
  
  
}
