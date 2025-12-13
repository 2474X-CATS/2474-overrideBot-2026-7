#include "vex.h"
#include "hood.h"

Hood *Hood::globalRef = nullptr;

void Hood::init()
{ 
  set<bool>("isOn", true); 
  if (RobotState::getStateOf("scoring_high")){ 
    set<double>("timestampHigh", Brain.Timer.time(vex::sec)); 
  }
}

void Hood::periodic()
{  
  if (shouldOpen() || (Brain.Timer.time(vex::sec) - get<double>("timestampHigh") <= 3)){  
    highPiston.close(); 
  } else { 
    highPiston.open();
  }

  if (isHolding())
  {
    holding = true;
  }
  else
  {
    if (holding)
    {
      if (hoodPiston.value() == 1)
        close();
      else
        open();
      holding = false;
      return;
    }
  }
  if (shouldClose())
  { 
    close();
  }
  else if (shouldOpen()) // Checks if the hood should close towards the hopper side
  { 
    open();
  } else {  

    open();
  }
}

void Hood::open()
{
  hoodPiston.open();
}

void Hood::close()
{
  hoodPiston.close();
}

void Hood::stop()
{
  close();
}

bool Hood::isHolding()
{
  return RobotState::getStateOf("toggling_hood");
};

bool Hood::shouldOpen()
{
  return RobotState::getStateOf("scoring_high");
};

bool Hood::shouldClose()
{
  return RobotState::getStateOf("intaking_to_hopper");
};

void Hood::updateTelemetry()
{
  return;
};
