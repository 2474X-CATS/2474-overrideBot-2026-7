#include "robot.h"
#include "telemetry.h"
#include "command.h"
#include "subsystem.h"
#include "vex.h"
using namespace vex;

/*
void timelyWait(long lastTimestamp, long timeInterval)
{
  long nextTimestamp = lastTimestamp + timeInterval;
  long waitTime = nextTimestamp - Brain.Timer.time();
  if (waitTime < 0)
    waitTime = 0;
  wait(waitTime, msec);
}
*/ 

Robot::Robot() {

};

void Robot::initialize()
{
  RobotState::initializeState();
  Subsystem::initSystems();
};

void Robot::driverControl()
{
  RobotState::setMode(ControlType::DRIVER);
  RobotState::manuallyModifyState("ready", true);
  RobotState::manuallyModifyState("is_drive_inverted", false);
  RobotState::manuallyModifyState("in_autonomous", false);
  //double timestamp;
  //timestamp = Brain.Timer.time();
  while (true)
  {
    Subsystem::updateSystems();
    wait(20, vex::msec);
    //timestamp = Brain.Timer.time();
  }
};

void Robot::runTelemetryThread()
{
  //int timestamp = Brain.Timer.time();
  while (true)
  {
    RobotState::updateState();
    RobotState::vibrate();
    Subsystem::refreshTelemetry();
    wait(20, vex::msec);
    //timestamp = Brain.Timer.time();
  }
};

void Robot::stopEverything()
{
  RobotState::setMode(ControlType::STOPPED);
  Subsystem::stopAll();
}

void Robot::setAutonomousCommand(std::vector<CommandInterface *> comm)
{
  autonomousCommand = comm;
};

void Robot::autonControl()
{
  RobotState::setMode(ControlType::MANUAL);
  RobotState::manuallyModifyState("in_autonomous", true);
  RobotState::manuallyModifyState("ready", true);
  for (CommandInterface *command : Robot::autonomousCommand)
  {
    command->run();
  }
};
