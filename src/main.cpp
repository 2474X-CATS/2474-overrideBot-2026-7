#include "vex.h"
#include "architecture/robot.h"
#include <iostream> 
#include "subsystems/forearm.h" 
#include "gui/graphics.h" 
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

int displayGraphData(){     
  DataSupplier desiredOutput; 
  DataSupplier actualOutput; 

  desiredOutput.directory = "forearm"; 
  desiredOutput.name = "requested_velocity"; 
  desiredOutput.label = "D(deg/sec)";  

  actualOutput.directory = "forearm"; 
  actualOutput.name = "current_velocity"; 
  actualOutput.label = "R(deg/sec)"; 

  
  Graph g = Graph( 
    "(D)esired & (R)eal Forearm Velocity", 
    { 
      desiredOutput, 
      actualOutput
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
  
  Forearm forearm = Forearm(); 
  
  //-------------------------------------------------------

  robot.initialize(); 

  //-------------------RUN PROTOCOLS HERE-------------------
  thread graph = thread(displayGraphData); 
  testDrive();

}
