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

void Subsystem::stopAll()
{
   for (Subsystem *system : systems)
   {
      system->stop();
   }
};

Subsystem *Subsystem::getSubsystem(int index)
{
   return systems.at(index);
}

//----------------------------------------------------------------------

vex::controller Controller = vex::controller(vex::controllerType::primary);

bool RobotState::axisesEnabled = false;
ControlType RobotState::mode = ControlType::STOPPED; 

string RobotState::vibrationCode = ":)";  

void RobotState::updateState()
{
   switch (mode)
   {
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

void RobotState::initializeState()
{
   Telemetry::inst.registerSubtable(
       "robot_state",
       {(EntrySet){"intaking_to_hopper", EntryType::BOOL},
        (EntrySet){"scoring_high", EntryType::BOOL},
        (EntrySet){"scoring_mid", EntryType::BOOL},
        (EntrySet){"scoring_low", EntryType::BOOL},
        (EntrySet){"matchloader_out", EntryType::BOOL},
        (EntrySet){"toggling_hood", EntryType::BOOL},
        (EntrySet){"toggling_descore", EntryType::BOOL},  
        (EntrySet){"mixing_hopper", EntryType::BOOL}, 
        (EntrySet){"reverse_intake", EntryType::BOOL}, 
        (EntrySet){"is_team_color_blue", EntryType::BOOL}, 
        (EntrySet){"loading_high", EntryType::BOOL}, 
        (EntrySet){"loading_mid", EntryType::BOOL}, 
        (EntrySet){"experiencing_jam", EntryType::BOOL}, 
      });
}

void RobotState::updateRegular()
{  

   /* 
   Whenever you are loading you are siphoning up cubes and preparing them to be scored on the high goal 
   - If the indexer detects blocks 
      THEN: The robot isn't in the loading state  
      IF NOT: The robot should be siphoning up cubes until cubes are detected
   */ 
   
   bool optOutOfLoading = Controller.ButtonB.pressing() && Controller.ButtonDown.pressing();
   
   manuallyModifyState("experiencing_jam", Telemetry::inst.getValueAt<bool>("indexer", "detects_jam") || Telemetry::inst.getValueAt<bool>("hopper", "detects_jam"));
   
   if (getStateOf("experiencing_jam")){ 
      setVibrationCode("---"); 
   } else if (Telemetry::inst.getValueAt<bool>("drivebase", "overheating")){ 
      setVibrationCode("-");
   } else { 
      disableVibrations();
   } 


   if (getStateOf("loading_high")){ 
      if (Telemetry::inst.getValueAt<bool>("indexer","detects_blocks_high") || optOutOfLoading || getStateOf("experiencing_jam")){ 
         manuallyModifyState("loading_high", false); 
         manuallyModifyState("scoring_high", false);
      } else {
         manuallyModifyState("scoring_high", true); 
      }
   } else if (getStateOf("loading_mid")) { 
      if (Telemetry::inst.getValueAt<bool>("indexer","detects_blocks_mid") || optOutOfLoading || getStateOf("experiencing_jam")){ 
         manuallyModifyState("loading_mid", false); 
         manuallyModifyState("scoring_mid", false);
      } else {
         manuallyModifyState("scoring_mid", true); 
      }
   } else { 
      if (Controller.ButtonA.pressing()){ 
       manuallyModifyState("loading_high", true);
      } else if (Controller.ButtonLeft.pressing()){ 
       manuallyModifyState("loading_mid", true);
      } else { 
       manuallyModifyState("scoring_high", Controller.ButtonR2.pressing()); 
       manuallyModifyState("scoring_mid", Controller.ButtonR1.pressing());
       manuallyModifyState("scoring_low", Controller.ButtonRight.pressing());   
       manuallyModifyState("intaking_to_hopper", Controller.ButtonY.pressing());
       manuallyModifyState("reverse_intake", Controller.ButtonB.pressing());   
       manuallyModifyState("toggling_hood", Controller.ButtonL1.pressing()); 
       manuallyModifyState("mixing_hopper", Controller.ButtonDown.pressing()); 
      }
   } 

   manuallyModifyState("matchloader_out", Controller.ButtonL2.pressing());
   manuallyModifyState("toggling_descore", Controller.ButtonX.pressing());
   
   
}

void RobotState::updateStopped()
{
   manuallyModifyState("intaking_to_hopper", false); 

   manuallyModifyState("scoring_high", false);
   manuallyModifyState("scoring_mid", false);
   manuallyModifyState("scoring_low", false); 


   manuallyModifyState("matchloader_out", false);
   manuallyModifyState("toggling_hood", false);
   manuallyModifyState("toggling_descore", false); 
   manuallyModifyState("mixing_hopper", false); 
   manuallyModifyState("reverse_intake", false);  

   manuallyModifyState("loading_high", false); 
   manuallyModifyState("loading_mid", false);

};

bool RobotState::getStateOf(string key)
{
   return Telemetry::inst.getValueAt<bool>("robot_state", key);
};

void RobotState::vibrate(){  
   if (vibrationCode != ":)")
      Controller.rumble(vibrationCode.c_str());
}; 

void RobotState::setVibrationCode(string code){ 
   vibrationCode = code;
}; 

void RobotState::disableVibrations(){ 
   vibrationCode = ":)";
}

int RobotState::getAxisState(AxisType axisType)
{
   int axisVal = 0;
   if (axisesEnabled)
   {
      switch (axisType)
      {
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

void RobotState::setMode(ControlType mod)
{
   mode = mod;
   updateStopped();
   if (mode == ControlType::DRIVER)
      axisesEnabled = true;
   else
   {
      axisesEnabled = false;
   };
};

void RobotState::manuallyModifyState(string key, bool val)
{
   Telemetry::inst.placeValueAt<bool>(val, "robot_state", key);
};

DummySystem GLOBAL_DUMMY;
