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
  /*
  while (!Competition.isEnabled()){ 
    this_thread::yield();
  } 
  */
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
      DrivePath::getCommand({105, TILE_SIZE_MM * 1.1, 210, TILE_SIZE_MM * 1.32, 90, TILE_SIZE_MM / 4 + 20}, true, true), 
      Score(Goal_Pos::HIGH_GOAL, 2500), 
      DriveLinear(-175, false)
    };
} 

vector<CommandInterface*> safeRunInverted(){ 
    return {  
      DrivePath::getCommand({79, TILE_SIZE_MM * 1.1, 330, TILE_SIZE_MM * 1.36, 92}, true, true),
      DriveLinear(TILE_SIZE_MM / 3 + 80, false),
      Score(Goal_Pos::HIGH_GOAL, 2500), 
      DriveLinear(-175, false)
    };
} 

vector<CommandInterface*> autonSkills(){ 
    return { 
      DriveLinear(TILE_SIZE_MM * 1.4, false), 
      TurnToHeading(180),  
      EnableMatchloader(true),
      DriveLinear(TILE_SIZE_MM * 0.8, true), 
      IntakeCubes(2000), 
      DriveLinear(-175, false), 
      TurnToHeading(2), 
      EnableMatchloader(false), 
      DriveLinear(TILE_SIZE_MM, true), 
      Score(Goal_Pos::HIGH_GOAL, 5000),  
      DriveLinear(-175, false), 
      TurnToHeading(270), 
      DriveLinear(TILE_SIZE_MM * 2.1, false), 
      TurnToHeading(180), 
      RamForward(1, 6000, true), 
    };
}



int main()
{

  vexcodeInit();
  // Initialize subsystems
  /*
    1: Configure auton
    2: Test auton
    3: Configure coding skills run
    4: Test coding skills run
    5: Run match [ Competitive | Driver-skills | Auton-skills ]
    6: Drive competition auton routine (recommended for test)
    7: Start competition match with commands as auton
    8: Free drive
  */

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
    autonSkills()
  );

}
