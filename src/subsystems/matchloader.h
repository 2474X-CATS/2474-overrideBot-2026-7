#ifndef __MATCHLOADER_H__
#define __MATCHLOADER_H__

#include "../architecture/subsystem.h"
#include "vex.h"

class Matchloader : public Subsystem
{

private:
  vex::pneumatics mlPiston;

public:
  using Subsystem::get;

  static Matchloader *globalRef;

  Matchloader() : Subsystem(
                      "matchloader",
                      {(EntrySet){"is_on", EntryType::BOOL}}),
                  mlPiston(vex::pneumatics(Brain.ThreeWirePort.F))
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