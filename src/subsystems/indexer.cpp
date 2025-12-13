#include "vex.h"
#include "indexer.h"

Indexer *Indexer::globalRef = nullptr;

double Indexer::ABSOLUTE_INDEXER_SPEED = 200;

array<int, 3> Indexer::RED_RGB = {210, 38, 48}; 
array<int, 3> Indexer::BLUE_RGB = {0,0,255};  

double Indexer::DISTANCE_THRESHOLD = 10;

void Indexer::init()
{
  indexerMotor.setBrake(vex::brake);   

  colorSensor.setLightPower(100); 
  colorSensor.setLight(vex::ledState::on); 
  colorSensor.integrationTime(20);
};

void Indexer::updateTelemetry()
{   
  set<bool>("detects_blocks_high", distanceSensor.objectDistance(vex::distanceUnits::cm) < DISTANCE_THRESHOLD);  
  set<bool>("detects_blocks_mid", colorSensor.isNearObject());   
  set<bool>("detects_jam", fabs(indexerMotor.velocity(vex::velocityUnits::rpm) - getExpectedVelocity()) > 1); 
};

void Indexer::periodic()
{  
  if (RobotState::getStateOf("loading_high"))  
  { 
    spinAtPercent(-0.2);
  }
  else if (RobotState::getStateOf("scoring_mid"))
  {
    spinAtPercent(1); //Below
  }  
  else if (RobotState::getStateOf("scoring_high"))
  {
    spinAtPercent(-1); // Above
  } 
  else if (RobotState::getStateOf("intaking_to_hopper")) 
  { 
     if (RobotState::getStateOf("color_sensitive")){  
      if (get<bool>("detects_blocks_mid") && (hasBlueBlock() != RobotState::getStateOf("is_team_color_blue"))){ 
        spinAtPercent(1);
      } else { 
        spinAtPercent(-0.1);
      } 
     } else { 
      spinAtPercent(-1);
     }
  } else
  {
    stop();
  }

};

bool Indexer::hasBlueBlock(){ 
  vex::optical::rgbc rgb = colorSensor.getRgb(true); 
  return (distBlue(rgb) < distRed(rgb));
} 

double Indexer::distBlue(vex::optical::rgbc rgb){ 
  return sqrt( pow((BLUE_RGB[0] - rgb.red), 2) + pow((BLUE_RGB[1] - rgb.green), 2) + pow((BLUE_RGB[2] - rgb.blue), 2));
} 

double Indexer::distRed(vex::optical::rgbc rgb){ 
  return sqrt( pow((RED_RGB[0] - rgb.red), 2) + pow((RED_RGB[1] - rgb.green), 2) + pow((RED_RGB[2] - rgb.blue), 2) );
}

double Indexer::getExpectedVelocity(){  
  return 0;
}; 


void Indexer::spinAtPercent(double percentage){ 
  indexerMotor.setVelocity(ABSOLUTE_INDEXER_SPEED * percentage, vex::velocityUnits::rpm); 
  indexerMotor.spin(vex::directionType::fwd);
}

void Indexer::stop()
{
  spinAtPercent(0);
}