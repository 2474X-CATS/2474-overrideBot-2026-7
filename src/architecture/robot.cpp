#include "robot.h"
#include "telemetry.h"
#include "command.h"
#include "subsystem.h"
#include "vex.h"
using namespace vex;

void timelyWait(long lastTimestamp, long timeInterval)
{
  long nextTimestamp = lastTimestamp + timeInterval;
  long waitTime = nextTimestamp - Brain.Timer.time();
  if (waitTime < 0)
    waitTime = 0;
  wait(waitTime, msec);
}

void drawCircleButton(int x, int y, string buttonName)
{
  color Color;
  uint32_t designatedColor;
  bool isBeingPressed = Telemetry::inst.getValueAt<bool>("system", "Controller/Button_" + buttonName);
  if (isBeingPressed)
  {
    designatedColor = Color.red;
  }
  else
  {
    designatedColor = Color.black;
  }
  Brain.Screen.drawCircle(x, y, 15, designatedColor);
}

void drawRectButton(int x, int y, int width, int height, string buttonName)
{
  color Color;
  uint32_t designatedColor;
  bool isBeingPressed = Telemetry::inst.getValueAt<bool>("system", "Controller/Button_" + buttonName);
  if (isBeingPressed)
  {
    designatedColor = Color.red;
  }
  else
  {
    designatedColor = Color.black;
  }
  Brain.Screen.drawRectangle(x - (width / 2), y - (height / 2), width, height, designatedColor);
}

void drawJoystick(int x, int y, string location)
{
  color Color;
  Brain.Screen.drawCircle(x, y, 30, Color.black);
  int horizontalPower = Telemetry::inst.getValueAt<int>("system", "Controller/Axis-Hori-" + location);
  int verticalPower = Telemetry::inst.getValueAt<int>("system", "Controller/Axis-Vert-" + location);
  Brain.Screen.drawCircle(x + (int)((30.0 / 2) * (horizontalPower / 100.0)), y - (int)((30.0 / 2) * (verticalPower / 100.0)), 30);
}

void displayGraphicalData()
{
  color Color;
  Brain.Screen.drawRectangle(70, 20, 355, 195, Color.rgb(100, 100, 100));

  drawJoystick(115, 70, "Left");
  drawJoystick(375, 70, "Right");

  drawCircleButton(330 + 22, 150, "A");
  drawCircleButton(330, 150 + 22, "B");
  drawCircleButton(330, 150 - 22, "X");
  drawCircleButton(330 - 22, 150, "Y");

  drawCircleButton(160 - 22, 150, "LEFT");
  drawCircleButton(160 + 22, 150, "RIGHT");
  drawCircleButton(160, 150 - 22, "UP");
  drawCircleButton(160, 150 + 22, "DOWN");

  drawRectButton(142, 12, 35, 20, "L2");
  drawRectButton(142, 25, 50, 30, "L1");
  drawRectButton(350, 12, 35, 20, "R2");
  drawRectButton(350, 25, 50, 30, "R1");
}

Robot::Robot() {

};

void Robot::initialize()
{
  registerSystemSubtables();
  Subsystem::initSystems();
};

void Robot::registerSystemSubtables()
{
  RobotState::initializeState();
};

void Robot::driverControl()
{ 
  RobotState::setMode(ControlType::DRIVER); 
  RobotState::manuallyModifyState("is_drive_inverted", true);
  double timestamp;
  timestamp = Brain.Timer.time();
  while (true)
  {
    Subsystem::updateSystems();
    timelyWait(timestamp, 20);
    timestamp = Brain.Timer.time();
  }
};

void Robot::runTelemetryThread(bool showGraphics)
{
  int timestamp = Brain.Timer.time();
  while (true)
  {
    RobotState::updateState(); 
    RobotState::vibrate();
    Subsystem::refreshTelemetry();
    if (showGraphics)
    {
      displayGraphicalData();
    }
    timelyWait(timestamp, 20);
    timestamp = Brain.Timer.time();
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
  RobotState::manuallyModifyState("in_autonomous", false); 

};
