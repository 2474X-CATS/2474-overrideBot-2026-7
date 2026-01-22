#include "intake.h"

double Intake::ABSOLUTE_CONVEYOR_SPEED = 200;
Intake *Intake::globalRef = nullptr;

void Intake::init()
{

   intakeConveyor.setBrake(vex::brakeType::brake);
   set<bool>("is_on", true);
   stop();
}

void Intake::periodic()
{
   if (RobotState::getStateOf("intaking"))
   { 
      intakeConveyor.setVelocity(-ABSOLUTE_CONVEYOR_SPEED * 1.5, vex::velocityUnits::rpm); // Inwards
      intakeConveyor.spin(vex::directionType::fwd);
   }  
   else if (RobotState::getStateOf("scoring_high")){ 
      intakeConveyor.setVelocity(-ABSOLUTE_CONVEYOR_SPEED * 1.2, vex::velocityUnits::rpm); // Inwards
      intakeConveyor.spin(vex::directionType::fwd);
   }
   else if (get<bool>("mid_scoring_engaged"))
   {   
      double delay = 150; 
      if (Brain.Timer.time(vex::msec) - get<double>("last_engaged_millis") >= delay){ 
          intakeConveyor.setVelocity(-ABSOLUTE_CONVEYOR_SPEED * 0.8, vex::velocityUnits::rpm); // Outwards
          intakeConveyor.spin(vex::directionType::fwd);
      } else { 
          stop();
      }
   }
   else if (RobotState::getStateOf("scoring_low"))
   {
      intakeConveyor.setVelocity(ABSOLUTE_CONVEYOR_SPEED, vex::velocityUnits::rpm); // Outwards
      intakeConveyor.spin(vex::directionType::fwd);
   }
   else
   {
      stop();
   }
}

void Intake::updateTelemetry()
{
   if (RobotState::getStateOf("scoring_mid"))
   {
      if (!get<bool>("mid_scoring_engaged"))
      {
         set<double>("last_engaged_millis", Brain.Timer.time(vex::msec));
         set<bool>("mid_scoring_engaged", true);
      }
   }
   else
   {
      set<bool>("mid_scoring_engaged", false);
   }
}

void Intake::stop()
{
   intakeConveyor.setVelocity(0, vex::percentUnits::pct);
   intakeConveyor.spin(vex::directionType::fwd);
}
