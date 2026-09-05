#include "vex.h"
#include "architecture/robot.h"
#include <iostream> 

#include "streams/supersystem.h" 
#include "subsystems/drivebase.h" 
#include "streams/odometry.h"
#include "subsystems/claw.h" 
#include "subsystems/elevator.h" 
#include "subsystems/forearm.h" 
#include "gui/graph.h" 

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

  drawLogo(RobotState::getStateOf("is_team_color_blue"));
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
  robot.runTelemetryThread();
}

int graphTableData(){ 
  DataSupplier zero; 
  DataSupplier error;  
   
  zero.directory = "graph";
  zero.name = "zero";
  zero.label = "O";
  
  error.directory = "graph";
  error.name = "error";
  error.label = "err(deg)";


  Graph g = Graph( 
    "Forearm Error", 
    { 
      zero, 
      error
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
      (EntrySet){"error", EntryType::DOUBLE}, 
      (EntrySet){"zero", EntryType::DOUBLE}
    }
  ); 

  Telemetry::inst.placeValueAt<double>(0, "graph", "zero");
   

  //--------------------SUBSYSTEM CREATION----------------- 
  
  Odometry odom = Odometry();
  Drivebase drive = Drivebase();
  SuperSystem ss = SuperSystem();
  Elevator elevator = Elevator();
  Forearm forearm = Forearm();
  Claw claw = Claw();
  
  //-------------------------------------------------------

  robot.initialize(); 

  //-------------------RUN PROTOCOLS HERE-------------------
  thread graphics = thread(graphTableData); 
  testDrive();
}
