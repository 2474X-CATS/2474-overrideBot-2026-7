#include "vex.h"
#include "architecture/robot.h"
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

int updateData(){   
  return 0;
}

//------------------------------>-------------------------------------------------------------------------------------------------------------------


int main()
{

  vexcodeInit();
  
   

  //--------------------SUBSYSTEM CREATION----------------- 
  
  
  
  //-------------------------------------------------------

  robot.initialize(); 

  //-------------------RUN PROTOCOLS HERE-------------------
  
  Telemetry::inst.registerSubtable( 
      "data", 
      { 
        (EntrySet){"output1", EntryType::DOUBLE}, 
        (EntrySet){"output2", EntryType::DOUBLE}
      }
  );  

  Telemetry::inst.placeValueAt<double>(5, "data", "output1"); 
  Telemetry::inst.placeValueAt<double>(700, "data", "output2");

  DataSupplier value1; 
  value1.directory = "data"; 
  value1.name = "output1"; 
  value1.label = "val1";

  DataSupplier value2; 
  value2.directory = "data"; 
  value2.name = "output2"; 
  value2.label = "val2"; 

  Graph graph = Graph(  
    "Values vs Inputs",
    { 
     value1, 
     value2
    } 
  ); 
  Sprite::frameLoop();
  
}
