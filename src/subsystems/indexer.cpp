#include "indexer.h" 

Indexer *Indexer::globalRef = nullptr;  

double Indexer::ABSOLUTE_INDEXER_SPEED = 200; 

void Indexer::init(){  
   indexerMotor.setBrake(vex::brakeType::brake);  
   colorSensor.setLight(vex::ledState::on);  
   colorSensor.integrationTime(20);
   colorSensor.setLightPower(100);
   set<bool>("is_on", true);     
   stop(); 
}  

void Indexer::periodic(){    
   bool ableToScore = true; 
   
   if (RobotState::getStateOf("in_autonomous")){ 
     if (RobotState::getStateOf("color_sensitive")){  
      ableToScore = get<bool>("detects_correct_color");
     }
   }   

   if (ableToScore && RobotState::getStateOf("scoring_high")){ 
     indexerMotor.setVelocity(-ABSOLUTE_INDEXER_SPEED * 0.9, vex::velocityUnits::rpm); 
     indexerMotor.spin(vex::directionType::fwd);  

   } else if (RobotState::getStateOf("scoring_mid")){    
     
     if (RobotState::getStateOf("in_autonomous") || RobotState::getStateOf("in_skills")){ 
       indexerMotor.setVelocity(ABSOLUTE_INDEXER_SPEED * 0.6, vex::velocityUnits::rpm);
     } else { 
       indexerMotor.setVelocity(ABSOLUTE_INDEXER_SPEED * 0.6, vex::velocityUnits::rpm);//Was 0.4
     } 
    
     indexerMotor.spin(vex::directionType::fwd);
   } else { 
     indexerMotor.setVelocity(0, vex::percentUnits::pct); 
     indexerMotor.spin(vex::directionType::fwd);
   }   

   if (ableToScore && RobotState::getStateOf("scoring_high")){ //Brain.Timer.time(vex::msec) - get<double>("last_long_goal_pressed") <= 1000){ 
     indexerHatch.open();
   } else { 
     indexerHatch.close();
   }  
} 

void Indexer::updateTelemetry(){  
   
   if (RobotState::getStateOf("scoring_high")){ 
    set<double>("last_long_goal_pressed", Brain.Timer.time(vex::msec));
   }  

   vex::color intolerableColor; 
   if (RobotState::getStateOf("is_team_color_blue")){ 
      intolerableColor = vex::red;
   } else { 
      intolerableColor = vex::blue;
   } 
  
   if (colorSensor.isNearObject()){ 
    lastBlockTimestamp = Brain.Timer.time(vex::msec);
   } 

   set<bool>("detects_block", Brain.Timer.time(vex::msec) - lastBlockTimestamp < 200); // 200 ms tolerance
   set<bool>("detects_correct_color", colorSensor.color() != intolerableColor); 
   
}  



void Indexer::stop(){  
   indexerMotor.setVelocity(0, vex::percentUnits::pct); 
   indexerMotor.spin(vex::directionType::fwd);
   indexerHatch.close();
}