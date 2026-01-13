#include "indexer.h" 

Indexer *Indexer::globalRef = nullptr;  

double Indexer::ABSOLUTE_INDEXER_SPEED = 200; 

void Indexer::init(){  
   indexerMotor.setBrake(vex::brakeType::brake); 
   set<bool>("is_on", true);     
   stop(); 
}  

void Indexer::periodic(){   
   if (RobotState::getStateOf("scoring_high")){ 
     indexerMotor.setVelocity(-ABSOLUTE_INDEXER_SPEED, vex::velocityUnits::rpm); 
     indexerMotor.spin(vex::directionType::fwd); 
   } else if (RobotState::getStateOf("scoring_mid")){  
     if (RobotState::getStateOf("in_autonomous")){ 
       indexerMotor.setVelocity(ABSOLUTE_INDEXER_SPEED * 0.25, vex::velocityUnits::rpm);
     } else { 
       indexerMotor.setVelocity(ABSOLUTE_INDEXER_SPEED * 0.5, vex::velocityUnits::rpm);
     }
     indexerMotor.spin(vex::directionType::fwd);
   } else { 
     indexerMotor.setVelocity(0, vex::percentUnits::pct); 
     indexerMotor.spin(vex::directionType::fwd);
   }  
   if (RobotState::getStateOf("scoring_high")){ 
     indexerHatch.close();
   } else { 
     indexerHatch.open();
   }  

} 

void Indexer::updateTelemetry(){ 
   return;
} 

void Indexer::stop(){  
   indexerMotor.setVelocity(0, vex::percentUnits::pct); 
   indexerMotor.spin(vex::directionType::fwd);
   indexerHatch.open();
}