#include "vex.h"
#include "hood.h"

Hood* Hood::globalRef = nullptr; 

void Hood::init()
{ 
    set<bool>("isOn", true);
}

void Hood::periodic()
{   
    if (isHolding()){  
      holding = true;
    } else {   
      if (holding){ 
        if (hoodPiston.value() == 1)
         close();   
        else
         open();   
        holding = false;
        return;
      }
    }
    if (shouldClose()){ 
      close();
    } else if (shouldOpen()) //Checks if the hood should close towards the hopper side 
    { 
      open(); 
    }
}

void Hood::open(){ 
  hoodPiston.open();
}  

void Hood::close(){ 
  hoodPiston.close();
}  

void Hood::stop(){ 
  close();
}

bool Hood::isHolding(){ 
  return RobotState::getStateOf("toggling_hood");
};

bool Hood::shouldOpen(){ 
  return RobotState::getStateOf("scoring_high");
};

bool Hood::shouldClose(){ 
  return RobotState::getStateOf("intaking_to_hopper");
};

void Hood::updateTelemetry()
{
    return;
};
