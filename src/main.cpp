#include "vex.h"
#include "architecture/robot.h" 
#include "subsystems/elevator.h" 
#include "gui/graph.h"
#include <iostream> 

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

  DataSupplier desired; 
  desired.directory = "test";
  desired.name = "desired_velocity"; 
  desired.label = "DV(mm/s)"; 

  DataSupplier reality; 
  reality.directory = "elevator";
  reality.name = "current_velocity"; 
  reality.label = "V(mm/s)";   
  
  DataSupplier position; 
  position.directory = "elevator"; 
  position.name = "current_height"; 
  position.label = "X(mm)";
  

  Graph g = Graph( 
     "Desired vs Real", 
     { 
      desired, 
      //reality, 
      position
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
  Telemetry::inst.registerSubtable( 
     "test", 
     { 
      (EntrySet){"desired_velocity", EntryType::DOUBLE}
     }
  );  

  Elevator elevator = Elevator();
  //-------------------------------------------------------

  robot.initialize(); 

  //-------------------RUN PROTOCOLS HERE-------------------  
  thread t = thread(initializeGraph);
  testDrive();

}
