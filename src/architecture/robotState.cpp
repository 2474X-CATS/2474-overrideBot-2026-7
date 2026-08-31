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
           (EntrySet){"awaiting_land", EntryType::BOOL},
           (EntrySet){"rise", EntryType::BOOL}, 
           (EntrySet){"fall", EntryType::BOOL}, 
           (EntrySet){"awaiting_claw_act", EntryType::BOOL}, 
           (EntrySet){"awaiting_flip", EntryType::BOOL},  
           (EntrySet){"k_claw_await", EntryType::BOOL},
           (EntrySet){"k_score", EntryType::BOOL}, 
           (EntrySet){"switch_score_mode", EntryType::BOOL},
           (EntrySet){"field_type_is_vex", EntryType::BOOL} // True = VEX, False = RECF
       }); 
   
   manuallyModifyState("field_type_is_vex", true);
}

void RobotState::updateRegular()
{   
   manuallyModifyState("awaiting_land", Controller1.ButtonY.pressing()); 
   manuallyModifyState("rise", Controller1.ButtonUp.pressing()); 
   manuallyModifyState("fall", Controller1.ButtonDown.pressing()); 
   
   
   if (Controller1.ButtonR1.pressing()){ 
      manuallyModifyState("k_claw_await", true);
   } else if (getStateOf("k_claw_await")){ 
      manuallyModifyState("k_claw_await", false);
      manuallyModifyState("awaiting_claw_act", true);
   }

   if (Controller1.ButtonX.pressing()){ 
      manuallyModifyState("k_score", true);
   } else if (getStateOf("k_score")){ 
      manuallyModifyState("k_score", false); 
      Telemetry::inst.placeValueAt<bool>(true, "ss_manager", "macro_requested");
   } 

   if (Controller1.ButtonL1.pressing()){ 
      manuallyModifyState("switch_score_mode", true);  
      /*
      if (Telemetry::inst.getValueAt<int>("ss_manager", "pickup_position") == SuperStructurePosition::GROUND){ 
         setVibrationCode("..");
      } else {
         setVibrationCode("."); 
      } 
      */
   } else if (getStateOf("switch_score_mode")){ 
      manuallyModifyState("switch_score_mode", false);  
      //disableVibrations(); 
      Telemetry::inst.placeValueAt<bool>(true, "ss_manager", "pickup_switch_requested");
   }  
   
   /*
   if (Telemetry::inst.getValueAt<bool>("forearm", "hold") || Telemetry::inst.getValueAt<bool>("elevator", "hold")){ 
         setVibrationCode("..");
   } else {
         disableVibrations();
   } 
   */

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

