#include "vex.h"
#include "architecture/robot.h"
#include <iostream> 

#include "streams/supersystem.h"
#include "subsystems/drivebase.h"
#include "streams/odometry.h"
#include "subsystems/claw.h" 
#include "subsystems/elevator.h"
#include "subsystems/toggle.h"
#include "streams/autoPrimer.h"

#include "gui/graph.h"  
//#include "gui/pathBoard.h" 


using namespace vex;

competition Competition;
Robot robot;  

//----------------PROTOCOLS TO RUN--------------------

void runTelemetry()
{
  robot.runTelemetryThread();
}

int scheduleCallbacks()
{
  Competition.autonomous([]()
                         { robot.autonControl(); });
  Competition.drivercontrol([]()
                            { robot.driverControl(false); });

  //drawLogo(RobotState::getStateOf("is_team_color_blue"));
  return 0;
}

void testDrive()
{   
  //drawLogo(RobotState::getStateOf("is_team_color_blue"));
  thread telemetryThread = thread(runTelemetry); 
  robot.driverControl(true);
}

void testAuto(vector<CommandInterface *> auton)
{
  robot.setAutonomousCommand(auton);
  thread telemThread = thread(runTelemetry);
  robot.autonControl();
  robot.driverControl(false);
}

void startCommandMatch()
{
  robot.configurateAutonomous();
  thread callBackTrigger = thread(scheduleCallbacks); 
  drawLogo(RobotState::getStateOf("is_team_color_blue"));
  robot.runTelemetryThread();
}

void startSkills(vector<CommandInterface*> auton){ 
   robot.setAutonomousCommand(auton);
   scheduleCallbacks();
   RobotState::manuallyModifyState("is_team_color_blue", false); 
   RobotState::manuallyModifyState("starting_left", true); 
   drawLogo(true);
   runTelemetry();
}

int runGraphics(){ 
  
  DataSupplier zero; 
  DataSupplier error;   
   
  zero.directory = "odometry";
  zero.name = "velocity_ms";
  zero.label = "mm/s"; 

  error.directory = "graph";
  error.name = "error";
  error.label = "Err";  
  
  Graph g = Graph( 
    "Data Display", 
    { 
      zero,
      //error
    }
  );   
  
  Sprite::frameLoop();
  
  return 0;

}

//------------------------------>-------------------------------------------------------------------------------------------------------------------


int main()
{

  vexcodeInit(); 
  
  Telemetry::inst.registerSubtable(
    "graph",
    { 
      (EntrySet){"zero", EntryType::DOUBLE}, 
      (EntrySet){"error", EntryType::DOUBLE}, 
    }
  ); 
  
  
  //Telemetry::inst.placeValueAt<double>(0, "graph", "zero");  

  //--------------------SUBSYSTEM CREATION----------------- 
   
  Odometry odom = Odometry();
  Drivebase drive = Drivebase();
  SuperSystem ss = SuperSystem();
  Claw claw = Claw();
  Elevator elevator = Elevator();
  Toggle toggle = Toggle();
  
  //------------------------------------------------------- 
  
  robot.initialize();
  
  //-------------------RUN PROTOCOLS HERE------------------- 
  
  RobotState::manuallyModifyState("starting_left", false); 
  RobotState::manuallyModifyState("is_team_color_blue", true);  
  
  testAuto(
    //auto_skills()
    //two_pin_alliance_left()
  );  
  
 //testDrive(); 
  

} 
