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

       });
}

void RobotState::updateRegular()
{  
   
}

void RobotState::updateStopped()
{

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
      case LEFT_HORIZONTAL:
         axisVal = Controller1.Axis4.position();
         break;
      case LEFT_VERTICAL:
         axisVal = Controller1.Axis3.position();
         break;
      case RIGHT_HORIZONTAL:
         axisVal = Controller1.Axis1.position();
         break;
      case RIGHT_VERTICAL:
         axisVal = Controller1.Axis2.position();
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