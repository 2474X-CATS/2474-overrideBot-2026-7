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
  

  double dist = hypot(metadata.positionX - endpoint[0], metadata.positionY - endpoint[1]); 

  double targetHeading = angleBetweenPts(endpoint[0], endpoint[1], metadata.positionX, metadata.positionY); 

  double angleDiff = angleDifference(targetHeading, metadata.angleHeading); 

  if (fabs(angleDiff) > 90 && cuttingCorners)
  {
    angleDiff = (180 - fabs(angleDiff)) * -1 * copysign(1, angleDiff); 
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
    angleDiff = toRadians(angleDiff);
    this->turningDirection = static_cast<int>(copysign(1, angleDiff));
    this->radius = dist / (sin(angleDiff));
    this->arcLength = fabs(this->radius * angleDiff);
    //this->radius /= 2;   
    
    this->endingHeading = angleSum(metadata.angleHeading, angleDiff);
  }

  metadata.motionConstants.maxVelocity = std::min<double>(
      metadata.motionConstants.maxVelocity,
      sqrt(metadata.maximumCentripetalAcceleration * this->radius));

  this->turnController = new errorcontroller(metadata.pidConstants);
  this->profile = new TrapezoidalMotionProfile(metadata.motionConstants, this->arcLength);

  activated = true;
}

void CirclePath::init(double timestamp)
{
  this->profile->init(startingVelocity, endingVelocity);
  this->lastTimestamp = timestamp;
  this->profile->setLastTimestamp(timestamp); 
  this->turnController->setReference(startingVelocity);
  this->turnController->setLastTimestamp(timestamp);
}

bool CirclePath::completed(double timestamp)
{
  return (timestamp - profile->getStartTime()) > profile->getTotalDuration();
}


double CirclePath::getAngularVelocity(double linearVelocity)
{
  if (straight)
  {
    return 0;
  } 

  return toDegrees(fabs((linearVelocity / this->radius))) * turningDirection;

}

PathFrameOutput CirclePath::calculateFrameOutput(double angularVelocity, double timestamp)
{
  PathFrameOutput output;
  if (activated)
  { 
    double correctiveOmega = turnController->calculate(angularVelocity, timestamp);
    
    output.linearVelocity = profile->generateSetpoint(timestamp).velocity * drivingDirection;
    output.angularVelocity = getAngularVelocity(output.linearVelocity);  

    turnController->setReference(output.angularVelocity);   

    output.angularVelocity += correctiveOmega; 
    
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