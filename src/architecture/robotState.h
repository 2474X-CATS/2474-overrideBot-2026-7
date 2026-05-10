#ifndef __ROBOT_STATE_H__ 
#define __ROBOT_STATE_H__ 

#include "telemetry.h" 

typedef enum
{
  LEFT_VERTICAL,
  LEFT_HORIZONTAL,
  RIGHT_VERTICAL,
  RIGHT_HORIZONTAL
} AxisType;

typedef enum
{
  DRIVER,
  MANUAL,
  STOPPED,
  INITIALIZATION
} ControlType;

class RobotState
{
private:
  static ControlType mode; // 1 is updating based on controller 2 is null state values 3 is manually set states (for auton)
  static bool axisesEnabled;

  static std::string vibrationCode;

  static void updateStopped();
  static void updateRegular();
  static void updateInitializing();

  static void disableVibrations();
  static void setVibrationCode(string code);

public:
  static void manuallyModifyState(string key, bool val);
  static bool getStateOf(string key);
  static bool getExternalState(string subtable, string key);
  static void setMode(ControlType control);
  static int getAxisState(AxisType axisType);

  static void initializeState();

  static void updateState();

  static void vibrate();
};


#endif