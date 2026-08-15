#include "vex.h"
#include "architecture/robot.h" 
#include "subsystems/drivebase.h" 
#include "streams/odometry.h"
#include <iostream>
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



int initializeGraph(){  
  
  Telemetry::inst.registerSubtable( 
    "blueprint", 
    { 
      (EntrySet){"velocity", EntryType::DOUBLE}
    }
  ); 

  DataSupplier real; 
  real.directory = "odometry";  
  real.name = "velocity_ms"; 
  real.label = "Real(m/s)"; 
  

  DataSupplier desired;  
  desired.directory = "blueprint"; 
  desired.name = "velocity";  
  desired.label = "BP(m/s)";
  

  Graph g = Graph( 
    "Velocity vs Desired Velocity", 
    { 
       real, 
       desired
    }
  ); 

  Sprite::frameLoop(); 

  return 0;
}

//------------------------------>-------------------------------------------------------------------------------------------------------------------


int main()
{

  vexcodeInit();
  
  
  //--------------------SUBSYSTEM CREATION----------------- 
  
  Odometry odom = Odometry(); 
  Drivebase drivebase = Drivebase();
  
  //-------------------------------------------------------

  robot.initialize(); 

  //-------------------RUN PROTOCOLS HERE-------------------
  
  //thread t = thread(initializeGraph); 

  testAuto( 
    { 
      TurnToHeading::getCommand(180), 
      DriveForward::getCommand(1000)
    }
  ); 

}
