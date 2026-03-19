#include "path.h"
#include "vex.h"

CirclePath::CirclePath(array<double, 2> endPoint, bool cuttingCorners, PathMetadata metadata) : CirclePath(endPoint, cuttingCorners)
{
  activate(metadata);
};

CirclePath::CirclePath(array<double, 2> endPoint, bool cuttingCorners)
{
  this->endpoint = endPoint;
  this->cuttingCorners = cuttingCorners;
}

void CirclePath::activate(PathMetadata metadata)
{

  this->projectedHeading = metadata.angleHeading;
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
  }
  else
  {
    angleDiff = angleDiff / 180 * M_PI;
    this->turningDirection = static_cast<int>(copysign(1, angleDiff));
    this->radius = dist / (2 * sin(angleDiff / 2));
    this->arcLength = fabs(this->radius * angleDiff) + 100;
    this->radius /= 2;
  }

  this->endingHeading = fmod(metadata.angleHeading + angleDiff + 360, 360);

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

double CirclePath::getAngularVelocity(double linearVelocity, double heading, double timestamp)
{

  if (straight)
  {
    return 0;
  }

  this->projectedHeading += this->lastOmega * ((timestamp - lastTimestamp) / 1000);

  double omega = fabs((linearVelocity / this->radius) / M_PI * 180) * turningDirection; // What the angular velocity should be
  this->lastOmega = omega;

  double error = this->projectedHeading - heading; // Now we can update the projected heading

  if (fabs(error) > 180)
  {
    error = (360 - fabs(error)) * -1 * copysign(1, error);
  }

  omega += turnController->calculate(error, timestamp); // Corrects for what the heading should be

  return omega;
}

PathFrameOutput CirclePath::calculateFrameOutput(double x, double y, double heading, double timestamp)
{
  PathFrameOutput output;
  if (activated)
  {
    output.linearVelocity = profile->generateSetpoint(timestamp).velocity * drivingDirection;
    output.angularVelocity = getAngularVelocity(output.linearVelocity, heading, timestamp);
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

void CirclePath::transformMetadata(PathMetadata data)
{
  data.angleHeading = getEndingHeading();
  data.positionX = getEndpointX();
  data.positionY = getEndpointY();
}