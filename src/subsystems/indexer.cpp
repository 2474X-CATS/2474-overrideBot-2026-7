#include "indexer.h" 

Indexer *Indexer::globalRef = nullptr;  


void Indexer::init(){   

   indexerMotor.setBrake(vex::brakeType::brake);   
   
   colorSensor.setLight(vex::ledState::on);
   colorSensor.setLightPower(100, vex::percentUnits::pct); 

   colorSensor.integrationTime(40); 

   hood.open(); 
   
   set<bool>("is_on", true);     
   stop();  

}  

void Indexer::periodic(){      


   if (RobotState::getStateOf("scoring_high")){   

     indexerMotor.setVelocity(-100, vex::velocityUnits::pct); 
     hood.open();   

   }  else if (RobotState::getStateOf("intaking") && !get<bool>("detects_block")){  

     indexerMotor.setVelocity(-100, vex::velocityUnits::pct);  
     hood.close(); 

   }  else if (RobotState::getStateOf("scoring_mid")){  
     
     if (RobotState::getStateOf("in_skills") || RobotState::getStateOf("in_autonomous")){ 
        indexerMotor.setVelocity(50, vex::velocityUnits::pct);
     } else { 
        indexerMotor.setVelocity(65, vex::velocityUnits::pct);
     }
     hood.open(); 

   } else if (RobotState::getStateOf("scoring_low")){   

     indexerMotor.setVelocity(100, vex::velocityUnits::pct);
     hood.close();   
     
   } else if (RobotState::getStateOf("swallow")) {  
       
     indexerMotor.setVelocity(100, vex::percentUnits::pct); 
     hood.open(); 

   } else {   

     indexerMotor.setVelocity(0, vex::velocityUnits::pct);  
     hood.close(); 

   }      

   indexerMotor.spin(vex::directionType::fwd);

} 

void Indexer::updateTelemetry(){  

   vex::color intolerableColor; 
   if (RobotState::getStateOf("is_team_color_blue")){ 
      intolerableColor = vex::red;
   } else { 
      intolerableColor = vex::blue;
   }  

   set<bool>("detects_block", colorSensor.isNearObject()); 
   set<bool>("detects_wrong_color", colorSensor.color() == intolerableColor);  

}  



void Indexer::stop(){  
   indexerMotor.setVelocity(0, vex::percentUnits::pct); 
   indexerMotor.spin(vex::directionType::fwd); 
   hood.close(); 
}