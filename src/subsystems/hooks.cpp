#include "hooks.h"

Hooks *Hooks::globalRef = nullptr;

void Hooks::init()
{
   set<bool>("is_on", true);
   stop();
}

void Hooks::periodic()
{
   hookPistonFront.set(RobotState::getStateOf("descore_f_in")); 
   hookPistonBack.set(RobotState::getStateOf("descore_b_in")); 
   lowGoalDescore.set(RobotState::getStateOf("low_descore_out"));
}

void Hooks::updateTelemetry()
{ 
   return;
}

void Hooks::stop()
{
   hookPistonFront.set(true); 
   hookPistonBack.set(true); 
   lowGoalDescore.set(false);
}