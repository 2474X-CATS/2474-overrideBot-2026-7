#include "vex.h"
#include "architecture/robot.h"
#include <iostream>

#include "subsystems/drivebase.h"
#include "subsystems/indexer.h"
#include "subsystems/hood.h"
#include "subsystems/hopper.h"
#include "subsystems/intake.h"
#include "subsystems/matchloader.h"
#include "subsystems/hooks.h" 

#include "commands.h"

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


vector<CommandInterface*> safeRun(){ 
    return { 
      DrivePath::getCommand({105, TILE_SIZE_MM * 1.1, 210, TILE_SIZE_MM * 1.3, 90, TILE_SIZE_MM / 4 + 120}, true, true),  
      Score(Goal_Pos::HIGH_GOAL, 2500), 
      DriveLinear(-175, false), 
    };
} 

vector<CommandInterface*> safeRunInverted(){ 
    return {  
      DrivePath::getCommand({78, TILE_SIZE_MM * 1.12, 330, TILE_SIZE_MM * 1.36, 92}, true, true),
      DriveLinear(TILE_SIZE_MM / 3 + 80, false),
      Score(Goal_Pos::HIGH_GOAL, 2500), 
      DriveLinear(-175, false), 
      TurnToHeading(270)
    };
} 

vector<CommandInterface*> parkOnly(){ 
   return { 
      RamForward(1, 4000, true)
   }; 
}



int main()
{

  vexcodeInit();
  // Initialize subsystems

  Drivebase drive = Drivebase(2,1/3); // Tile location right 1 up 1
  Intake intake;
  Matchloader matchloader;
  Indexer indexer;
  Hood hood;
  Hopper hopper;
  Hooks hooks;

  robot.initialize();  
  
  RobotState::manuallyModifyState("is_team_color_blue", true);
  RobotState::manuallyModifyState("color_sensitive", false); 
  
  startCommandMatch( 
    safeRunInverted()
  ); 

}
