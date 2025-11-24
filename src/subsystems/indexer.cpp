#include "vex.h"
#include "indexer.h"

Indexer *Indexer::globalRef = nullptr;

double Indexer::ABSOLUTE_INDEXER_SPEED = 200;

void Indexer::init()
{
  Telemetry::inst.placeValueAt<double>(indexerMotor.temperature(), "Motor_Temps", "IndexerMotor");
  set<bool>("isOn", true);
};

void Indexer::updateTelemetry()
{
  return;
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
  return RobotState::getStateOf("scoring_high") || RobotState::getStateOf("intaking_to_hopper");
}

bool Indexer::shouldSpinUnder()
{
  return RobotState::getStateOf("scoring_mid");
}

void Indexer::spinOver()
{
  indexerMotor.setVelocity(-ABSOLUTE_INDEXER_SPEED, vex::velocityUnits::rpm);
  indexerMotor.spin(vex::directionType::fwd);
}

void Indexer::spinUnder()
{
  indexerMotor.setVelocity(ABSOLUTE_INDEXER_SPEED * 0.75, vex::velocityUnits::rpm);
  indexerMotor.spin(vex::directionType::fwd);
}

void Indexer::stop()
{
  indexerMotor.setVelocity(0, vex::percentUnits::pct);
  indexerMotor.spin(vex::directionType::fwd);
}
