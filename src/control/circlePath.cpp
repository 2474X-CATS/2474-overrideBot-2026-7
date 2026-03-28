#include "path.h"
#include "vex.h"

CirclePath::CirclePath(BiarcEnum biarc, PathMetadata metadata) : CirclePath(biarc)
{
  activate(metadata);
};

CirclePath::CirclePath(BiarcEnum biarc)
{
  this->endpoint = biarc.endpoint;
  this->cuttingCorners = biarc.cuttingCorners;
}

void CirclePath::activate(PathMetadata metadata)
{
  //this->projectedHeading = metadata.angleHeading;

  double dist = hypot(metadata.positionX - endpoint[0], metadata.positionY - endpoint[1]); 

  double targetHeading = fmod((atan2(endpoint[1] - metadata.positionY, endpoint[0] - metadata.positionX)) / M_PI * 180 + 360, 360); 

  double angleDiff = targetHeading - metadata.angleHeading; 

  if (fabs(angleDiff) > 180)
  {
    angleDiff = (360 - fabs(angleDiff)) * -1 * copysign(1, angleDiff);
  }
  if (fabs(angleDiff) > 90 && cuttingCorners)
  {
    angleDiff = fmod((180 - fabs(angleDiff)) * -1 * copysign(1, angleDiff), 180); 
    drivingDirection = -1;
  }
  if (fabs(angleDiff) < 1e-6)
  {
    straight = true;
    this->radius = 1e7;
    this->arcLength = dist;  
    this->endingHeading = metadata.angleHeading;
  }
  else
  {     

    angleDiff = angleDiff / 180 * M_PI;
    this->turningDirection = static_cast<int>(copysign(1, angleDiff));
    this->radius = dist / (2 * sin(angleDiff / 2));
    this->arcLength = fabs(this->radius * angleDiff);
    this->radius /= 2;  
    
    this->endingHeading = fmod(metadata.angleHeading + angleDiff + 360, 360);
  }

  metadata.motionConstants.maxVelocity = std::min<double>(
      metadata.motionConstants.maxVelocity,
      sqrt(metadata.maximumCentripetalAcceleration * this->radius));

  this->turnController = new pidcontroller(metadata.pidConstants, 0);
  this->profile = new TrapezoidalMotionProfile(metadata.motionConstants, this->arcLength);

  activated = true;
}

void CirclePath::init(double timestamp)
{
  this->profile->init(startingVelocity, endingVelocity);
  this->lastTimestamp = timestamp;
  this->profile->setLastTimestamp(timestamp);
  this->turnController->setLastTimestamp(timestamp);
}

bool CirclePath::completed(double timestamp)
{
  return (timestamp - profile->getStartTime()) > profile->getTotalDuration();
}

double CirclePath::getDesiredHeading(double positionX, double positionY){ 
  double distance = hypot(positionX - endpoint.at(0), positionY - endpoint.at(1));  
  double angleDiff = (2 * asin(distance / (2 * radius))) / M_PI * 180;  
  return fmod(endingHeading - angleDiff + 360, 360);
}

double CirclePath::calculateAngleCorrectionOutput(double positionX, double positionY, double heading, double timestamp){ 
  if (straight){ 
    return 0;
  }
  double desiredHeading = getDesiredHeading(positionX, positionY); 
  double angleDiff = desiredHeading - heading; 
  if (angleDiff > 180 || angleDiff < -180){ 
    angleDiff = (360 - fabs(angleDiff)) * -1 * copysign(1, angleDiff);
  } 
  return turnController->calculate(angleDiff, timestamp);
}

double CirclePath::getAngularVelocity(double linearVelocity)
{
  if (straight)
  {
    return 0;
  }

  return fabs((linearVelocity / this->radius) / M_PI * 180) * turningDirection; 

}

PathFrameOutput CirclePath::calculateFrameOutput(double x, double y, double heading, double timestamp)
{
  PathFrameOutput output;
  if (activated)
  {
    output.linearVelocity = profile->generateSetpoint(timestamp).velocity * drivingDirection;
    output.angularVelocity = getAngularVelocity(output.linearVelocity) + calculateAngleCorrectionOutput(x, y, heading, timestamp);
    this->lastTimestamp = timestamp;
  }
  return output;
}

void CirclePath::setStartingVelocity(double velocity)
{
  this->startingVelocity = velocity;
}

void CirclePath::setEndingVelocity(double velocity)
{
  this->endingVelocity = velocity;
}

double CirclePath::getMaximumVelocity()
{
  return activated ? this->profile->getMaxVelocity() : -1;
}

double CirclePath::getEndingHeading()
{
  return this->endingHeading;
}

double CirclePath::getEndpointX()
{
  return endpoint.at(0);
}

double CirclePath::getEndpointY()
{
  return endpoint.at(1);
}

void CirclePath::linkLeftToRight(CirclePath *path1, CirclePath *path2)
{
  double consensusVelocity = std::min<double>(path1->getMaximumVelocity(), path2->getMaximumVelocity());
  path1->setEndingVelocity(consensusVelocity);
  path2->setStartingVelocity(consensusVelocity);
}

void CirclePath::transformMetadata(PathMetadata *data)
{
  data->angleHeading = getEndingHeading();
  data->positionX = getEndpointX();
  data->positionY = getEndpointY();
}