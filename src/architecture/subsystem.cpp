#include "subsystem.h"
#include "vex.h" 

vector<Subsystem *> Subsystem::systems;

Subsystem::Subsystem(string tableLabel, vector<EntrySet> entryNames) : label(tableLabel)
{
   Subsystem::systems.push_back(this);
   Telemetry::inst.registerSubtable(this->label, entryNames);
};

void Subsystem::updateSystems()
{
   for (Subsystem *system : systems)
   {
      system->periodic();
   }
};

void Subsystem::initSystems()
{
   for (Subsystem *system : systems)
   {
      system->init();
   }
};

void Subsystem::refreshTelemetry()
{
   for (Subsystem *system : systems)
   {
      system->updateTelemetry();
   }
}; 

void Subsystem::stopAll(){ 
  for (Subsystem *system : systems)
   {
      system->stop();
   }
};   

Subsystem* Subsystem::getSubsystem(int index){ 
  return systems.at(index);
} 


//----------------------------------------------------------------------

vex::controller Controller = vex::controller(vex::controllerType::primary); 

bool RobotState::axisesEnabled = false; 
ControlType RobotState::mode = ControlType::STOPPED;

void RobotState::updateState(){  
  switch (mode){ 
   case DRIVER: 
      updateRegular(); 
      break;
   case STOPPED: 
      updateStopped();   
      break;
   case MANUAL: 
   default: 
      break;
  }
};   

void RobotState::initializeState(){  
   Telemetry::inst.registerSubtable( 
      "robot_state",  
      { 
            (EntrySet){"intaking_to_hopper", EntryType::BOOL}, 
            (EntrySet){"scoring_high", EntryType::BOOL},
            (EntrySet){"scoring_mid", EntryType::BOOL}, 
            (EntrySet){"scoring_low", EntryType::BOOL}, 
            (EntrySet){"matchloader_out", EntryType::BOOL}, 
            (EntrySet){"toggling_hood", EntryType::BOOL},
            (EntrySet){"toggling_descore", EntryType::BOOL}
      }
   );    
}


void RobotState::updateRegular(){ 
   Telemetry::inst.placeValueAt<bool>(Controller.ButtonY.pressing(), "robot_state", "intaking_to_hopper");  
   Telemetry::inst.placeValueAt<bool>(Controller.ButtonR2.pressing(), "robot_state", "scoring_high");
   Telemetry::inst.placeValueAt<bool>(Controller.ButtonR1.pressing(), "robot_state", "scoring_mid");
   Telemetry::inst.placeValueAt<bool>(Controller.ButtonB.pressing(), "robot_state", "scoring_low");
   Telemetry::inst.placeValueAt<bool>(Controller.ButtonL2.pressing(), "robot_state", "matchloader_out");
   Telemetry::inst.placeValueAt<bool>(Controller.ButtonL1.pressing(), "robot_state", "toggling_hood");
   Telemetry::inst.placeValueAt<bool>(Controller.ButtonX.pressing(), "robot_state", "toggling_descore");
}


void RobotState::updateStopped(){ 
   Telemetry::inst.placeValueAt<bool>(false, "robot_state", "intaking_to_hopper");  
   Telemetry::inst.placeValueAt<bool>(false, "robot_state", "scoring_high");
   Telemetry::inst.placeValueAt<bool>(false, "robot_state", "scoring_mid");
   Telemetry::inst.placeValueAt<bool>(false, "robot_state", "scoring_low");
   Telemetry::inst.placeValueAt<bool>(false, "robot_state", "matchloader_out");
   Telemetry::inst.placeValueAt<bool>(false, "robot_state", "toggling_hood");
   Telemetry::inst.placeValueAt<bool>(false, "robot_state", "toggling_descore");
}; 


bool RobotState::getStateOf(string key){ 
   return Telemetry::inst.getValueAt<bool>("robot_state", key); 
};  


int RobotState::getAxisState(AxisType axisType){ 
   int axisVal; 
   if (axisesEnabled){ 
    switch (axisType){ 
      case LEFT_HORIZONTAL:   
        axisVal = Controller.Axis4.position();
        break;
      case LEFT_VERTICAL:  
        axisVal = Controller.Axis3.position(); 
        break; 
      case RIGHT_HORIZONTAL:   
        axisVal = Controller.Axis1.position();
        break; 
      case RIGHT_VERTICAL:   
        axisVal = Controller.Axis2.position(); 
        break;
    } 
   }
   return axisVal;
};

void RobotState::setMode(ControlType mod){ 
   mode = mod;   
   updateStopped(); 
   if (mode == ControlType::DRIVER) 
      axisesEnabled = true; 
   else { 
      axisesEnabled = false;
   };
};  


void RobotState::manuallyModifyState(string key, bool val){ 
   Telemetry::inst.placeValueAt<bool>(val,"robot_state",key); 
};  


DummySystem GLOBAL_DUMMY;

