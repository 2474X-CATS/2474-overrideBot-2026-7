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
#include "helpers/autoConfig.h"

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
  while (!Competition.isEnabled())
    this_thread::yield();
  robot.runTelemetryThread(true);
}

void driveCommandMatch(std::vector<CommandInterface *> commandGroup)
{
  robot.setAutonomousCommand(commandGroup);  // Registers the autonomous routine
  thread telemThread = thread(runTelemetry); // Start data logging
  robot.autonControl();                      // Runs the autonomous command
  robot.driverControl();                     // Free drive
}

vector<CommandInterface *> closedSide()
{
  return {
      AlignWithLocation(Zones::NAT_MID, TILE_SIZE_MM, PathType::EUCLIDEAN, true),
      DriveLinear(TILE_SIZE_MM * 0.9, false),
      Score(Goal_Pos::MID_GOAL, 1000),
      AlignWithLocation(Zones::NAT_ML_LEFT, TILE_SIZE_MM * 0.75, PathType::EUCLIDEAN, false),
      EnableMatchloader(true),
      DriveLinear(TILE_SIZE_MM * 0.875, false),
      IntakeCubes(3000),
      AlignWithLocation(Zones::NAT_HIGH_LEFT, TILE_SIZE_MM * 0.5, PathType::EUCLIDEAN, false),
      EnableMatchloader(false),
      DriveLinear(TILE_SIZE_MM * 0.35, false),
      Score(Goal_Pos::HIGH_GOAL, 2000)};
}

vector<CommandInterface *> selfSufficent()
{
  return {
      AlignWithLocation(Zones::NAT_ML_LEFT, TILE_SIZE_MM, PathType::EUCLIDEAN, false),
      EnableMatchloader(true),
      DriveLinear(TILE_SIZE_MM * 0.85, false),
      IntakeCubes(2000),
      AlignWithLocation(Zones::NAT_HIGH_LEFT, TILE_SIZE_MM * 0.5, PathType::EUCLIDEAN, false),
      EnableMatchloader(false),
      DriveLinear(TILE_SIZE_MM * 0.4, false),
      Score(Goal_Pos::HIGH_GOAL, 2000),
      DriveLinear(-TILE_SIZE_MM * 0.3, false),
      AlignWithLocation(Zones::NAT_MID, TILE_SIZE_MM, PathType::EUCLIDEAN, true),
      DriveLinear(TILE_SIZE_MM * 0.85, false),
      Score(Goal_Pos::MID_GOAL, 2000),
      DriveLinear(-TILE_SIZE_MM * 0.3, false),
      AlignWithLocation(Zones::NAT_LOW, TILE_SIZE_MM, PathType::EUCLIDEAN, true),
      DriveLinear(TILE_SIZE_MM * 0.75, false),
      Score(Goal_Pos::LOW_GOAL, 2500)};
}

vector<CommandInterface *> shortRun()
{
  return {
      AlignWithLocation(Zones::NAT_MID, TILE_SIZE_MM, PathType::MANHATTAN_YX, true),
      DriveLinear(TILE_SIZE_MM * 0.85, false),
      Score(Goal_Pos::MID_GOAL, 2000),
      DriveLinear(-TILE_SIZE_MM * 0.3, false),
      AlignWithLocation(Zones::NAT_LOW, TILE_SIZE_MM, PathType::EUCLIDEAN, true),
      DriveLinear(TILE_SIZE_MM * 0.75, false),
      Score(Goal_Pos::LOW_GOAL, 2500)};
};

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

  Drivebase drive = Drivebase(2.75, 0.75); // Tile location right 1 up 1

  Intake intake;
  Matchloader matchloader;
  Indexer indexer;
  Hood hood;
  Hopper hopper;
  Hooks hooks;

  robot.initialize(); 

  runAutonomousMaker();
  
  /*
  driveCommandMatch(
      {  
         RamForward(1, 2000)
      }
  );  
  */ 


  

}
