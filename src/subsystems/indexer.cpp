#include "vex.h"
#include "indexer.h"

Indexer *Indexer::globalRef = nullptr;

double Indexer::ABSOLUTE_INDEXER_SPEED = 200;

double Indexer::RED_HUE = 0; 
double Indexer::BLUE_HUE = 0;  

double Indexer::DISTANCE_THRESHOLD = 10;

void Indexer::init()
{
  indexerMotor.setBrake(vex::brake);  

  colorSensor.setLightPower(50); 
  colorSensor.setLight(vex::ledState::on); 

};

void Indexer::updateTelemetry()
{  
  double currentHue = colorSensor.hue(); 

  set<bool>("detects_blocks_high", currentHue == RED_HUE || currentHue == BLUE_HUE);  
  set<bool>("detects_blocks_mid", distanceSensor.objectDistance(vex::distanceUnits::cm) < DISTANCE_THRESHOLD);
  set<bool>("is_block_blue", get<bool>("detects_blocks") && currentHue == BLUE_HUE);     
  set<bool>("detects_jam", fabs(indexerMotor.velocity(vex::velocityUnits::rpm) - getExpectedVelocity()) > 1); 

};

void Indexer::periodic()
{
  if (shouldSpinOver())
  {
    spinOver();
  }
  else if (shouldSpinUnder())
  {
    spinUnder();
  }
  else
  {
    stop();
  }

};


bool Indexer::shouldSpinOver()
{
  return RobotState::getStateOf("scoring_high") || (RobotState::getStateOf("intaking_to_hopper") && sensesValidBlock());
}

bool Indexer::shouldSpinUnder()
{
  return RobotState::getStateOf("scoring_mid") || (RobotState::getStateOf("intaking_to_hopper") && sensesInvalidBlock());
} 

bool Indexer::sensesValidBlock(){  
  return get<bool>("detects_blocks") && (get<bool>("is_block_blue") == RobotState::getStateOf("is_team_color_blue"));
}; 

bool Indexer::sensesInvalidBlock(){ 
  return get<bool>("detects_blocks") && (get<bool>("is_block_blue") != RobotState::getStateOf("is_team_color_blue"));
};

double Indexer::getExpectedVelocity(){  
  double output;
  if (shouldSpinOver()){ 
    output =  -ABSOLUTE_INDEXER_SPEED;
  } else if (shouldSpinUnder()){ 
    output = ABSOLUTE_INDEXER_SPEED; 
  } else { 
    output = 0;
  } 
  return output;
}; 

void Indexer::spinOver()
{
  indexerMotor.setVelocity(-ABSOLUTE_INDEXER_SPEED, vex::velocityUnits::rpm);
  indexerMotor.spin(vex::directionType::fwd);
}

void Indexer::spinUnder()
{
  indexerMotor.setVelocity(ABSOLUTE_INDEXER_SPEED, vex::velocityUnits::rpm);
  indexerMotor.spin(vex::directionType::fwd);
}

void Indexer::stop()
{
  indexerMotor.setVelocity(0, vex::percentUnits::pct);
  indexerMotor.spin(vex::directionType::fwd);
}