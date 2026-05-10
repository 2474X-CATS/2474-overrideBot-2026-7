#include "robot.h"
//#include "telemetry.h"
#include "command.h"
#include "subsystem.h" 
#include "dataStream.h"
//#include "vex.h" 


using namespace vex;



Robot::Robot() {

};

void Robot::initialize()
{
  RobotState::initializeState();
  Subsystem::initSystems(); 
  DataStream::initializeAll();
};

void Robot::driverControl(bool odometryEnabled)
{
  RobotState::setMode(ControlType::DRIVER);
  RobotState::manuallyModifyState("in_autonomous", false);
  while (true)
  {
    Subsystem::updateSystems();
    wait(20, vex::msec);
  }
};

void Robot::runTelemetryThread()
{
  while (true)
  { 
    DataStream::refreshAll();
    RobotState::updateState();
    Subsystem::refreshTelemetry();  
    RobotState::vibrate();
    wait(20, vex::msec);
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
  vex::wait(20, msec);  
  for (CommandInterface *command : Robot::autonomousCommand)
  {
    command->run();
  }
}; 

void Robot::configurateAutonomous(){   

  vector<Routine> routines = generateRoutinePool(); 

  ColorPicker colorChooser = ColorPicker(135 + 115, 200);
  SidePicker sidePicker = SidePicker(235 + 115, 200);
  RoutineCatalog catalog = RoutineCatalog(10, 10, getOptionVector(routines));
  
  AutonDisplay(250, 80, &catalog, &sidePicker);
  ExitBlock(250, 10); 

  Sprite::frameLoop();

  int sideIndex;
  int autonIndex = catalog.getCurrentAuto().index;

  if (sidePicker.getIsLeft())
  { 
    sideIndex = 0;
  }
  else
  {
    sideIndex = 1;
  }

  setAutonomousCommand(routines.at(autonIndex).autos.at(sideIndex));
  RobotState::manuallyModifyState("is_team_color_blue", colorChooser.getIsBlue());
}
