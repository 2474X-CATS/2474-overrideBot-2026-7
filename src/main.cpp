#include "vex.h"
#include "architecture/robot.h"
#include <iostream>
/*
#include "subsystems/drivebase.h" 
#include "subsystems/indexer.h"
#include "subsystems/intake.h"
#include "subsystems/matchloader.h"
#include "subsystems/hooks.h"  
*/ 

#include "subsystems/wedge.h"
#include "subsystems/hooks.h" 
#include "commands.h"
#include "helpers/autonInit.h"

using namespace vex;

competition Competition;
Robot robot;

void runTelemetry()
{
  robot.runTelemetryThread(false); 
} 

void pauseUntilReady(){ 
  while (!RobotState::getStateOf("ready")){ 
    this_thread::yield();
  }
}

int scheduleCallbacks(){  
  double startingTime = Brain.Timer.time(vex::sec); 

  while (Brain.Timer.time(vex::sec) - startingTime < 30){ 
     this_thread::yield();
  }  

  RobotState::manuallyModifyState("ready", true); 

  Competition.autonomous([]()
                         { robot.autonControl(); });
  Competition.drivercontrol([]()
                            { robot.driverControl(); });  
  drawLogo();
  return 0;
}

void freeDrive()
{  
  drawLogo();
  RobotState::manuallyModifyState("ready", true); 
  thread telemetryThread = thread(runTelemetry);
  robot.driverControl();
}

void configurateAutonomous(vector<CommandInterface*> leftSide, vector<CommandInterface*> rightSide){ 
  ColorPicker colorChooser = ColorPicker(220-125,200-50);  
  SidePicker sidePicker = SidePicker(360-125,200-50);  
  ExitBlock(160, 65);
  
  Sprite::frameLoop();   

  drawLogo();
  
  if (sidePicker.getIsLeft()){ 
    robot.setAutonomousCommand(leftSide);
  } else { 
    robot.setAutonomousCommand(rightSide); 
  }    

  RobotState::manuallyModifyState("is_team_color_blue", colorChooser.getIsBlue());
}  

void testAuto(vector<CommandInterface*> auton){ 
  robot.setAutonomousCommand(auton);
  thread telemThread = thread(runTelemetry);   
  pauseUntilReady(); 
  drawLogo();
  robot.autonControl();                      
  robot.driverControl();
} 

void startCommandSkillsMatch(vector<CommandInterface*> commandGroup){  
  robot.setAutonomousCommand(commandGroup); 
  thread callbackWait = thread(scheduleCallbacks);
  robot.runTelemetryThread(false);
}

void startCommandCompetitiveMatch(vector<CommandInterface *> leftCommandGroup, vector<CommandInterface *> rightCommandGroup)
{
  configurateAutonomous(leftCommandGroup, rightCommandGroup);
  thread callbackWait = thread(scheduleCallbacks);
  robot.runTelemetryThread(false); 
}

void driveCommandMatch(vector<CommandInterface *> leftCommandGroup, vector<CommandInterface*> rightCommandGroup)
{  
  configurateAutonomous(leftCommandGroup, rightCommandGroup);
  thread telemThread = thread(runTelemetry);   
  pauseUntilReady();
  robot.autonControl();                      
  robot.driverControl();                     
}


vector<CommandInterface*> closed_side_left(){ 
   return { 
       DriveToLocation(Zones::NAT_MID, TILE_SIZE_MM * 1.35, PathType::EUCLIDEAN, true),   
       DeployMatchloader::getCommand(true), 
       DriveForwardForTime::getCommand(0.4, 500, true),
       IntakeCubes::getCommand(225),   
       DeployMatchloader::getCommand(false),
       ModifyRobotState::getCommand("is_drive_inverted", true),  
       TurnToLocation(Zones::NAT_MID), 
       DrivePath::getCommand({TILE_SIZE_MM * 0.45}, false, false),  
       ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 2000),    
       ModifyRobotState::getCommand("is_drive_inverted", false),  
       DrivePath::getCommand({50}, false, false),
       DriveToLocation(Zones::NAT_ML_LEFT, TILE_SIZE_MM * 0.925, PathType::EUCLIDEAN, false),  
       TurnToLocation(Zones::NAT_ML_LEFT),  
       DeployMatchloader::getCommand(true), 
       WaitFor::getCommand(500), 
       DriveForwardForTime::getCommand(0.1, 1300, true),
       IntakeCubes::getCommand(500),   
       ModifyRobotState::getCommand("is_drive_inverted", true),   
       DriveToLocation(Zones::NAT_HIGH_LEFT, TILE_SIZE_MM * 0.2, PathType::EUCLIDEAN, false), 
       ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 2000)
   };
}


int main()
{ 

  vexcodeInit();
  
  Drivebase drive = Drivebase(0,0); // Tile location right 0 up 0
  Intake intake; 
  Indexer indexer;
  Matchloader matchloader;  
  Hooks hooks;     
  Wedge wedge; 
  
  robot.initialize();   

  RobotState::manuallyModifyState("color_sensitive", false); 
  
  RobotState::manuallyModifyState("descore_out",true);  
  RobotState::manuallyModifyState("matchloader_out", false); 

  matchloader.periodic();
  hooks.periodic();  

  //---------------------------------------------------------------
  
  testAuto({ 
    DriveToLocation(Zones::NAT_ML_LEFT, TILE_SIZE_MM * 0.75, PathType::EUCLIDEAN, true)
  }); 

  

}
