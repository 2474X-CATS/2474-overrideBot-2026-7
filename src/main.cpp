#include "vex.h"
#include "architecture/robot.h" 
#include "subsystems/drivebase.h" 
#include "streams/odometry.h" 
#include "subsystems/elevator.h"
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
  drawLogo(RobotState::getStateOf("is_team_color_blue"));
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



//------------------------------>-------------------------------------------------------------------------------------------------------------------


int main()
{

  vexcodeInit();
  
  //--------------------SUBSYSTEM CREATION----------------- 
  
  Drivebase drive = Drivebase(); 
  Odometry odom = Odometry(); 
  Elevator elev = Elevator();
  
  //-------------------------------------------------------

  robot.initialize(); 

  //-------------------RUN PROTOCOLS HERE------------------- 
  testDrive();
  

}
