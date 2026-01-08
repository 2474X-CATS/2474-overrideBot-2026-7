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

void freeDrive()
{
  thread telemetryThread = thread(runTelemetry);
  robot.driverControl();
}

void startCommandMatch(std::vector<CommandInterface *> commandGroup)
{
  robot.setAutonomousCommand(commandGroup);
  Competition.autonomous([]()
                         { robot.autonControl(); });
  Competition.drivercontrol([]()
                            { robot.driverControl(); });  
  while (!Competition.isEnabled()){ 
    this_thread::yield();
  }  
  robot.runTelemetryThread(false);
}

void driveCommandMatch(std::vector<CommandInterface *> commandGroup)
{
  robot.setAutonomousCommand(commandGroup);  // Registers the autonomous routine
  thread telemThread = thread(runTelemetry); // Start data logging
  robot.autonControl();                      // Runs the autonomous command
  robot.driverControl();                     // Free drive
}

void testGraphics(){  
  Field(275, 10);
  ColorPicker(220,200);  
  SidePicker(360,200);
  Sprite::frameLoop(); 
}


int main()
{ 

  vexcodeInit();
  
  Drivebase drive = Drivebase(1,1); // Tile location right 0 up 0
  Intake intake; 
  Indexer indexer;
  Matchloader matchloader;  
  Hooks hooks;     
  Wedge wedge; 
  
  robot.initialize();   

  RobotState::manuallyModifyState("is_team_color_blue", true);
  RobotState::manuallyModifyState("color_sensitive", false); 
  
  driveCommandMatch({ 
     ModifyRobotState::getCommand("is_drive_inverted", true), 
     DeployMatchloader::getCommand(false),  
     TurnToSetpoint::getCommand(TILE_SIZE_MM * 2, TILE_SIZE_MM * 2), 
     DrivePath::getCommand({TILE_SIZE_MM * 1.2}, false, true)
  });
  

}
