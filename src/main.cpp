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
#include "helpers/location.h" 
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


void startCommandMatch(std::vector<CommandInterface*> commandGroup){  
  robot.setAutonomousCommand(commandGroup);
  Competition.autonomous([](){robot.autonControl();}); 
  Competition.drivercontrol([](){robot.driverControl();});  
  while (!Competition.isEnabled()) 
     this_thread::yield();
  robot.runTelemetryThread(true);
}  

void driveCommandMatch(std::vector<CommandInterface*> commandGroup){   
  robot.setAutonomousCommand(commandGroup); //Registers the autonomous routine
  thread telemThread = thread(runTelemetry); //Start data logging
  robot.autonControl(); //Runs the autonomous command
  robot.driverControl(); //Free drive
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
  Drivebase drive = Drivebase(1,1); //Tile location right 1 up 1    
  Intake intake;  
  Matchloader matchloader;   
  Indexer indexer;  
  Hood hood; 
  Hopper hopper;     
  Hooks hooks; 

  robot.initialize();  
  
  freeDrive(); 

}
