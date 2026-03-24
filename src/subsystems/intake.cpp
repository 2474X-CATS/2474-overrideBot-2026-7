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
      intakeConveyor.setVelocity(-ABSOLUTE_CONVEYOR_SPEED * 2, vex::velocityUnits::rpm); // Inwards
      intakeConveyor.spin(vex::directionType::fwd);
   }
   else if (RobotState::getStateOf("scoring_high"))
   {
      intakeConveyor.setVelocity(-ABSOLUTE_CONVEYOR_SPEED * 0.725, vex::velocityUnits::rpm); // Inwards
      intakeConveyor.spin(vex::directionType::fwd);
   }
   else if (get<bool>("mid_scoring_engaged"))
   {
      double delay;
      double outputSpeed;
      double backTrackSpeed;

      if (RobotState::getStateOf("in_skills"))
      {
         delay = 225;
         outputSpeed = ABSOLUTE_CONVEYOR_SPEED * 0.8;
         backTrackSpeed = ABSOLUTE_CONVEYOR_SPEED * 0.3;
      }
      else
      {
         delay = 225;
         outputSpeed = ABSOLUTE_CONVEYOR_SPEED * 0.8;
         backTrackSpeed = ABSOLUTE_CONVEYOR_SPEED * 0.3;
      }

      if (Brain.Timer.time(vex::msec) - get<double>("last_engaged_millis") >= delay)
      {
         intakeConveyor.setVelocity(-outputSpeed, vex::velocityUnits::rpm); // Outwards
      }
      else
      {
         intakeConveyor.setVelocity(backTrackSpeed, vex::velocityUnits::rpm);
      }

      intakeConveyor.spin(vex::directionType::fwd);
   }
   else if (RobotState::getStateOf("scoring_low") || RobotState::getStateOf("outtaking"))
   {
      intakeConveyor.setVelocity(ABSOLUTE_CONVEYOR_SPEED, vex::velocityUnits::rpm); // Outwards
      intakeConveyor.spin(vex::directionType::fwd);
   }
   else
   {
      stop();
   }

   if (RobotState::getStateOf("scoring_low"))
   {
      lowGoalFilter.close();
   }
   else
   {
      lowGoalFilter.open();
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
   lowGoalFilter.open();
}
