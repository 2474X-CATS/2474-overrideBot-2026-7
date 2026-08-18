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
      (EntrySet){"desired_position", EntryType::DOUBLE}, 
      (EntrySet){"progress", EntryType::DOUBLE}, 
      (EntrySet){"desired_velocity", EntryType::DOUBLE}
    }
  ); 

  DataSupplier target; 
  target.directory = "blueprint";  
  target.name = "desired_position"; 
  target.label = "Setp(mm)"; 
  

  DataSupplier progress;  
  progress.directory = "blueprint"; 
  progress.name = "progress";
  progress.label = "Pos(mm)"; 

  DataSupplier speed;
  speed.directory = "odometry"; 
  speed.name = "velocity_ms"; 
  speed.label = "V(mm/s)";   

  DataSupplier desiredSpeed; 
  desiredSpeed.directory = "blueprint"; 
  desiredSpeed.name = "desired_velocity"; 
  desiredSpeed.label = "DV(mm/s)"; 


  Graph g = Graph( 
    "Position vs Desired Position", 
    { 
       target, 
       progress, 
       //speed, 
       //desiredSpeed
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
  Telemetry::inst.placeValueAt<bool>(true, "odometry", "oriented_c"); 
  
  thread t = thread(initializeGraph);
  testDrive();

  /*
  testAuto( 
    { 
      TurnToHeading::getCommand(270), 
      TurnToHeading::getCommand(180), 
      TurnToHeading::getCommand(90), 
      TurnToHeading::getCommand(0)
    }
  ); 
  */  

  /*
  testAuto( 
    { 
      DriveToSetpoint::getCommand(TILE_SIZE_MM * 3, TILE_SIZE_MM * 1, RouteType::EUCLIDEAN), 
      DriveToSetpoint::getCommand(TILE_SIZE_MM * 3, TILE_SIZE_MM * 4, RouteType::EUCLIDEAN), 
      DriveToSetpoint::getCommand(TILE_SIZE_MM * 1, TILE_SIZE_MM * 4, RouteType::EUCLIDEAN), 
      DriveToSetpoint::getCommand(TILE_SIZE_MM * 5, TILE_SIZE_MM * 4, RouteType::EUCLIDEAN)
    }
  );  
  */

}
