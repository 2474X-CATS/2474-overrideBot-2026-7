#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <vector> 
#include "../autos.h"   
#include "../gui/autonInit.h"

using std::vector;

class CommandInterface; 

class Robot
{

private:
   vector<CommandInterface *> autonomousCommand; 

public:
   Robot();
   
   void configurateAutonomous();

   void initialize(); // All subsysytems in the subsystem list as a result of their instantiation are initialized here (Motor prep / sensor calibration)

   void driverControl(bool odometryEnabled); // Drive the robot using input

   void autonControl(); // Runs a series of commands that have been prior set by the "setAutonomusCommand" method

   void setAutonomousCommand(vector<CommandInterface *> comm); // Sets up the autonomous command that is to be ran in autoncontrol:
                                                               // Like building blocks (Put one command on top of the other)
   void stopEverything();

   void runTelemetryThread(); // A constant loop of information logging fom the subsystems (drivebase, intake, etc),
                              // and the system (controller axises, buttons) happening on a different thread
}; 




#endif