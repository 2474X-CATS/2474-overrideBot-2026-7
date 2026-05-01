#include "intake.h"

Intake *Intake::globalRef = nullptr;

void Intake::init()
{
   intakeConveyor.setBrake(vex::brakeType::brake); 
   idlerMotor.setBrake(vex::brakeType::brake);
   set<bool>("is_on", true);
   stop();
}

void Intake::synchronizedMotorRequest(double value, vex::velocityUnits units){ 
   intakeConveyor.setVelocity(value, units); 
   idlerMotor.setVelocity(value, units);  
   intakeConveyor.spin(vex::directionType::fwd); 
   idlerMotor.spin(vex::directionType::fwd);
}

void Intake::periodic()
{   

   if (RobotState::getStateOf("intaking")){   

      synchronizedMotorRequest(100, vex::velocityUnits::pct); 
   
   } else if (get<bool>("delay_active")){  
      
      double delay;  
      bool delayIntake = false;

      if (RobotState::getStateOf("scoring_high")){ 
        delay = 75;  //100
      } else if (RobotState::getStateOf("scoring_mid")){
        delay = 100; 
        delayIntake = true;
      } else { 
        delay = 0;
      }   
      
      if (Brain.Timer.time() - get<double>("time_since_activated") < delay){  
        idlerMotor.setVelocity(-100, vex::velocityUnits::pct);   
        if (delayIntake){ 
          intakeConveyor.setVelocity(-100, vex::velocityUnits::pct);
        } else {  
          intakeConveyor.setVelocity(100, vex::velocityUnits::pct);
        } 
      } else {  
        //idlerMotor.setVelocity(100, vex::velocityUnits::pct);    
        if (delayIntake){  
          if (RobotState::getStateOf("in_autonomous") || RobotState::getStateOf("in_skills")){  
            idlerMotor.setVelocity(25, vex::velocityUnits::pct);
            intakeConveyor.setVelocity(25, vex::velocityUnits::pct);
          } else { 
            intakeConveyor.setVelocity(65, vex::velocityUnits::pct); 
            idlerMotor.setVelocity(65, vex::velocityUnits::pct);
          }
        } else { 
          intakeConveyor.setVelocity(100, vex::velocityUnits::pct);  
          idlerMotor.setVelocity(100, vex::velocityUnits::pct);
        }
      }     
      
      intakeConveyor.spin(vex::directionType::fwd); 
      idlerMotor.spin(vex::directionType::fwd);

   } else if (RobotState::getStateOf("scoring_low") || RobotState::getStateOf("swallow")){ 
      synchronizedMotorRequest(-75, vex::velocityUnits::pct);
   }
   else
   {
      stop();
   }
}

void Intake::updateTelemetry()
{
  if (RobotState::getStateOf("scoring_high") || RobotState::getStateOf("scoring_mid")){
    if (!get<bool>("delay_active")){ 
      set<bool>("delay_active", true); 
      set<double>("time_since_activated", Brain.Timer.time());
    } 
  } else { 
    set<bool>("delay_active", false);
  }  

  if (RobotState::getStateOf("quiet")){ 
    set<double>("idler_efficiency", 100);
  } else { 
    set<double>("idler_efficiency", idlerMotor.efficiency(vex::percentUnits::pct) );
  }  
  
}

void Intake::stop()
{
   intakeConveyor.stop(); 
   idlerMotor.stop();
}
