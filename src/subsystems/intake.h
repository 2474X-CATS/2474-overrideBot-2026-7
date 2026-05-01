#ifndef __INTAKE_H__
#define __INTAKE_H__

#include "../architecture/subsystem.h"
#include "vex.h"

class Intake : public Subsystem
{

private:
  vex::motor intakeConveyor; 
  vex::motor idlerMotor;   

public:
  using Subsystem::get;

  static Intake *globalRef;
  
  Intake() : Subsystem(
                 "intake",
                 { 
                  (EntrySet){"is_on", EntryType::BOOL}, 
                  (EntrySet){"delay_active", EntryType::BOOL},  
                  (EntrySet){"time_since_activated", EntryType::DOUBLE},  
                  (EntrySet){"idler_efficiency", EntryType::DOUBLE}
                 } 
             ),
             intakeConveyor(vex::motor(vex::PORT8)), 
             idlerMotor(vex::motor(vex::PORT7)) 
             //intakeChain(vex::motor_group(intakeConveyor, idlerMotor))
  {
    globalRef = this;
  }

  void init() override;
  void periodic() override;
  void updateTelemetry() override;
  void stop() override; 

  void synchronizedMotorRequest(double value, vex::velocityUnits units);

protected: 
  using Subsystem::set;
};

#endif