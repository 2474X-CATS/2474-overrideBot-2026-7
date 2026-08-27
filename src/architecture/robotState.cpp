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
           (EntrySet){"fall", EntryType::BOOL} 

       });
}

void RobotState::updateRegular()
{   
  
   manuallyModifyState("awaiting_land", Controller1.ButtonY.pressing());
   manuallyModifyState("rise", Controller1.ButtonUp.pressing()); 
   manuallyModifyState("fall", Controller1.ButtonDown.pressing());

   //--------------------------------------------------------------------------- 
   
  
   
    


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

