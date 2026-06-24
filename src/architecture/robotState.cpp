#include "robotState.h"
#include "vex.h" 

vex::controller Controller1 = vex::controller(vex::controllerType::primary);
vex::controller Controller2 = vex::controller(vex::controllerType::partner);

bool RobotState::axisesEnabled = false;
ControlType RobotState::mode = ControlType::INITIALIZATION;

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
   case INITIALIZATION:
      updateInitializing();
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
       {
           (EntrySet){"in_autonomous", EntryType::BOOL}, 
           (EntrySet){"inverted", EntryType::BOOL}, 
           (EntrySet){"is_team_color_blue", EntryType::BOOL},   
           
           (EntrySet){"requested_macro", EntryType::BOOL},   

           (EntrySet){"requested_drop", EntryType::BOOL},  
           (EntrySet){"awaiting_drop", EntryType::BOOL}, 

           (EntrySet){"requested_land", EntryType::BOOL},  
           (EntrySet){"awaiting_land", EntryType::BOOL}, 

           (EntrySet){"requested_flip", EntryType::BOOL}, 
           (EntrySet){"awaiting_flip", EntryType::BOOL}



       });
}

void RobotState::updateRegular()
{  
   if (Controller2.ButtonR2.pressing()){ //Macro
      manuallyModifyState("requested_macro", true);
   } else { 
      if (getStateOf("requested_macro")){ 
          Telemetry::inst.placeValueAt<bool>(true, "ss_manager", "macro_requested");  
          manuallyModifyState("requested_macro", false);
      }
   }

   //--------------------------------------------------------------------------

   if (Controller2.ButtonA.pressing()){ //Drop
      manuallyModifyState("requested_drop", true);
   } else { 
      if (getStateOf("requested_drop")){ 
          manuallyModifyState("awaiting_drop", true); //Turned off by claw itself
          manuallyModifyState("requested_drop", false);
      }
   }  

   //--------------------------------------------------------------------------- 
   
   if (Controller2.ButtonB.pressing()){ //Land
      manuallyModifyState("requested_land", true);
   } else { 
      if (getStateOf("requested_land")){ 
          manuallyModifyState("awaiting_land", true); //Turned off by the arm itself 
          manuallyModifyState("requested_land", false);
      }
   } 
   
    //---------------------------------------------------------------------------

   if (Controller2.ButtonX.pressing()){ //Flip
      manuallyModifyState("requested_flip", true);
   } else { 
      if (getStateOf("requested_flip")){ 
          manuallyModifyState("awaiting_flip", true); //Turned off by claw itself
          manuallyModifyState("requested_flip", false);
      }
   }


};

void RobotState::updateStopped() {

};

void RobotState::updateInitializing()
{
   return;
}

bool RobotState::getStateOf(string key)
{
   return Telemetry::inst.getValueAt<bool>("robot_state", key);
};

bool RobotState::getExternalState(string subtable, string key)
{
   return Telemetry::inst.getValueAt<bool>(subtable, key);
};

void RobotState::vibrate()
{
   if (vibrationCode != ":)")
      Controller1.rumble(vibrationCode.c_str());
};

void RobotState::setVibrationCode(string code)
{
   vibrationCode = code;
};

void RobotState::disableVibrations()
{
   vibrationCode = ":)";
}

int RobotState::getAxisState(AxisType axisType)
{
   int axisVal = 0;
   if (axisesEnabled)
   {
      switch (axisType)
      {
      case M_LEFT_HORIZONTAL:
         axisVal = Controller1.Axis4.position();
         break;
      case M_LEFT_VERTICAL:
         axisVal = Controller1.Axis3.position();
         break;
      case M_RIGHT_HORIZONTAL:
         axisVal = Controller1.Axis1.position();
         break;
      case M_RIGHT_VERTICAL:
         axisVal = Controller1.Axis2.position();
         break; 
      case S_LEFT_HORIZONTAL:
         axisVal = Controller2.Axis4.position();
         break;
      case S_LEFT_VERTICAL:
         axisVal = Controller2.Axis3.position();
         break;
      case S_RIGHT_HORIZONTAL:
         axisVal = Controller2.Axis1.position();
         break;
      case S_RIGHT_VERTICAL:
         axisVal = Controller2.Axis2.position();
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

