#include "intake.h" 

double Intake::ABSOLUTE_CONVEYOR_SPEED = 200;
Intake *Intake::globalRef = nullptr; 

void Intake::init(){  
   
   intakeConveyor.setBrake(vex::brakeType::brake);
   set<bool>("is_on", true);     
   stop(); 
}  

void Intake::periodic(){ 

   if (RobotState::getStateOf("intaking") || RobotState::getStateOf("scoring_high")){ 
      intakeConveyor.setVelocity(-ABSOLUTE_CONVEYOR_SPEED, vex::velocityUnits::rpm); //Inwards
      intakeConveyor.spin(vex::directionType::fwd); 
   } else if (RobotState::getStateOf("scoring_mid")){ 
      intakeConveyor.setVelocity(-ABSOLUTE_CONVEYOR_SPEED * 0.8, vex::velocityUnits::rpm); //Outwards
      intakeConveyor.spin(vex::directionType::fwd);
   } else if (RobotState::getStateOf("scoring_low")){ 
      intakeConveyor.setVelocity(ABSOLUTE_CONVEYOR_SPEED, vex::velocityUnits::rpm); //Outwards
      intakeConveyor.spin(vex::directionType::fwd);
   } else { 
      intakeConveyor.setVelocity(0, vex::percentUnits::pct); 
      intakeConveyor.spin(vex::directionType::fwd);
   } 
   
   if (RobotState::getStateOf("intaking") || RobotState::getStateOf("scoring_mid")){ 
     intakeStopper.open(); 
   } else { 
     intakeStopper.close(); 
   }  
   

} 

void Intake::updateTelemetry(){ 
   return;
} 

void Intake::stop(){ 
   intakeConveyor.setVelocity(0, vex::percentUnits::pct); 
   intakeConveyor.spin(vex::directionType::fwd); 
   intakeStopper.open();
}
