#ifndef __HOOKS_H__
#define __HOOKS_H__

#include "../architecture/subsystem.h"
#include "vex.h"

class Hooks : public Subsystem
{

private:
  vex::pneumatics hookPistonFront; 
  vex::pneumatics hookPistonBack; 
  vex::pneumatics lowGoalDescore;

public:
  using Subsystem::get;

  static Hooks *globalRef;
  
  Hooks() : Subsystem(
                "hooks",
                {(EntrySet){"is_on", EntryType::BOOL}}),
            hookPistonFront(vex::pneumatics(Brain.ThreeWirePort.C)), 
            hookPistonBack(vex::pneumatics(Brain.ThreeWirePort.D)), 
            lowGoalDescore(vex::pneumatics(Brain.ThreeWirePort.E)) 
  {
    globalRef = this;
  }

  void init() override;
  void periodic() override;
  void updateTelemetry() override;
  void stop() override;

protected:
  using Subsystem::set;
};

#endif