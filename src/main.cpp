#include "vex.h"
#include "architecture/robot.h"
#include <iostream> 

#include "streams/supersystem.h" 
#include "subsystems/drivebase.h" 
#include "streams/odometry.h"
#include "subsystems/claw.h" 
#include "subsystems/elevator.h" 
#include "subsystems/forearm.h"  
#include "subsystems/intake.h" 
#include "streams/autoPrimer.h" 

#include "gui/graph.h"  
#include "gui/pathBoard.h" 


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

int runGraphics(){ 
  /*
  Point p1;
  p1.x = TILE_SIZE_MM; 
  p1.y = TILE_SIZE_MM;  
  p1.heading = 135;

  Point p2;
  p2.x = TILE_SIZE_MM * 4; 
  p2.y = TILE_SIZE_MM * 4;  
  p2.heading = 90;

  Point p3;
  p3.x = TILE_SIZE_MM * 1; 
  p3.y = TILE_SIZE_MM * 4; 
  
  Arc traj = Arc(p1, p2); 

  PathBoard pBoard = PathBoard(traj, 15);
  Sprite::frameLoop();

  return 0; 
  */
  
  DataSupplier zero; 
  DataSupplier error;   
   
  zero.directory = "graph";
  zero.name = "zero";
  zero.label = "Goal";
  
  error.directory = "graph";
  error.name = "error";
  error.label = "Err";  

  Graph g = Graph( 
    "Forearm PID", 
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
      (EntrySet){"zero", EntryType::DOUBLE}, 
      (EntrySet){"error", EntryType::DOUBLE}, 
    }
  ); 

  //Telemetry::inst.placeValueAt<double>(0, "graph", "zero");


  //--------------------SUBSYSTEM CREATION----------------- 
   
  
  Odometry odom = Odometry(); 
  Drivebase drive = Drivebase(); 
  
  Intake intake = Intake(); 
  SuperSystem ss = SuperSystem();  
  
  Forearm forearm = Forearm(); 
  Elevator elevator = Elevator();
  
  Claw claw = Claw();    
  
  
  //-------------------------------------------------------
  
  robot.initialize(); 
  //wait(5000, vex::msec); 
  //-------------------RUN PROTOCOLS HERE-------------------
  //thread graphics = thread(runGraphics); 

  testDrive();  



} 
