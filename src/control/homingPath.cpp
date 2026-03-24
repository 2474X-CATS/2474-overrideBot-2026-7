#include "path.h"
#include "vex.h"

double HomingPath::TUNED_LDIST = hypot(ROBOT_LENGTH_MM * 1.5, ROBOT_WIDTH_MM * 1.5);
double HomingPath::TUNED_L_SCALE = 1;
double HomingPath::OPTIMUM_TOLERANCE = 100;

HomingPath::HomingPath(BezierCurve *curve, TrapezoidConstants motionConstants, PIDConstants pidconstants, double lookAheadDistance, double k_scale, double maxCentripAccel, double distTolerance)
{

   this->curve = curve;

   this->lDist = lookAheadDistance;
   this->maxCentripetalAcceleration = maxCentripAccel;
   this->k_scale = k_scale;

   if (distTolerance != -1)
   {
      this->distanceTolerance = distTolerance;
   }

   this->endpoint = curve->generatePoint(1);

   this->curveProfile = new TrapezoidalMotionProfile(motionConstants, curve->getPathLength());
   this->turnController = new pidcontroller(pidconstants, 0);
}

HomingPath::HomingPath(array<array<double, 2>, 3> points, TrapezoidConstants motionConstants, PIDConstants pidconstants, double maxCentripAccel)
    : HomingPath(new BezierCurve(points), motionConstants, pidconstants, TUNED_LDIST, TUNED_L_SCALE, maxCentripAccel, OPTIMUM_TOLERANCE) {};

HomingPath::HomingPath(array<array<double, 2>, 2> points, PathMetadata metadata) : HomingPath(
                                                                                       array<array<double, 2>, 3>{
                                                                                           array<double, 2>{metadata.positionX, metadata.positionY},
                                                                                           points[0],
                                                                                           points[1]},
                                                                                       metadata.motionConstants,
                                                                                       metadata.pidConstants,
                                                                                       metadata.maximumCentripetalAcceleration) {};

PathFrameOutput HomingPath::calculateFrameOutput(double x, double y, double heading, double timestamp)
{

   PathFrameOutput res;
   BezierReferencePoint refPoint = findReferencePoint(x, y, getOptimumLookaheadDist(lastVelocity));

   double deltaTime = (timestamp - lastTimestamp) / 1000;

   double dist = sqrt(pow(x - refPoint.point[0], 2) + pow(y - refPoint.point[1], 2)); // Distance between the setpoint and the current position
   dist = std::max<double>(dist, 1e-6);

   double desiredHeading = (atan2(refPoint.point[1] - y, refPoint.point[0] - x) + M_PI) / (2 * M_PI) * 360; // At that point the robot would face the setpoint
   double angleDiff = desiredHeading - heading;                                                             // How much the robot needed to turn in order to face the setpoint
   if (fabs(angleDiff) > 180)
   {
      angleDiff = (-1 * copysign(1, angleDiff)) * (360 - fabs(angleDiff));
   }
   double angularVelocity = turnController->calculate(angleDiff, Brain.Timer.time());

   double linearVelocity = std::min<double>(lastVelocity + (getAcceleration() * deltaTime), curveProfile->getMaxVelocity()); // Only add a frame of acceleration
   linearVelocity = std::min<double>(linearVelocity, deriveMaxVelocity(fabs(dist / std::max<double>(2 * sin(angleDiff / 360 * M_PI), 1e-6))));

   res.angularVelocity = angularVelocity;
   res.linearVelocity = linearVelocity;

   lastVelocity = linearVelocity;
   lastTimestamp = timestamp;

   updateAnchorPoint(refPoint.progressT);

   return res;
}

BezierCurve *HomingPath::getCurve()
{
   return curve;
}

BezierReferencePoint HomingPath::findReferencePoint(double x, double y, double lookAhead)
{ // TO-DO
   BezierReferencePoint res;

   double currentT = pathProgress;

   array<double, 2> coordinates;

   curve->generatePoint(pathProgress, coordinates);

   double fallbackT = pathProgress;
   double smallestDist = sqrt(pow(x - coordinates[0], 2) + pow(y - coordinates[1], 2));

   double t_interval = (1.0 / 500);

   currentT += t_interval;

   double currentDist = 0;

   bool withinRadialZone = false;

   while (currentT < 1)
   {
      curve->generatePoint(currentT, coordinates);
      currentDist = sqrt(pow(x - coordinates[0], 2) + pow(y - coordinates[1], 2));

      if (smallestDist == -1 || currentDist < smallestDist)
      {
         smallestDist = currentDist;
         fallbackT = currentT;
      }

      if (currentDist > lookAhead)
      {
         if (withinRadialZone)
         {
            break;
         }
      }
      else
      {
         withinRadialZone = true;
      }
      currentT += t_interval;
   }

   if (!withinRadialZone)
   {
      currentT = fallbackT;
   }

   curve->generatePoint(currentT, coordinates);

   res.point = coordinates;
   res.progressT = fallbackT; // The closest t-value

   return res;
}

double HomingPath::getOptimumLookaheadDist(double velocity)
{
   return lDist + (k_scale * velocity);
}

double HomingPath::deriveMaxVelocity(double radius)
{
   return sqrt(maxCentripetalAcceleration * radius);
}

void HomingPath::init(double timestamp)
{
   this->lastTimestamp = timestamp;
   this->curveProfile->setLastTimestamp(timestamp);
   this->turnController->setLastTimestamp(timestamp);
}

TrapezoidalMotionProfile *HomingPath::getProfile()
{
   return curveProfile;
}

void HomingPath::updateAnchorPoint(double closestT)
{
   array<double, 2> p1 = curve->generatePoint(pathProgress);
   array<double, 2> p2 = curve->generatePoint(closestT);

   distanceTraveled += hypot(p1[0] - p2[0], p1[1] - p2[1]);

   pathProgress = closestT;
}

double HomingPath::getPathProgress()
{
   return pathProgress;
}

double HomingPath::getAcceleration()
{

   if (distanceTraveled < curveProfile->getAccelDist())
   {

      return curveProfile->getMaxAcceleration();
   }
   else if (distanceTraveled < (curveProfile->getCruiseDist() + curveProfile->getAccelDist()))
   {

      return std::min<double>(
          (pow(curveProfile->getMaxVelocity(), 2) - pow(lastVelocity, 2)) / (2 * ((curveProfile->getCruiseDist() + curveProfile->getAccelDist()) - distanceTraveled)),
          curveProfile->getMaxAcceleration());
   }
   else
   {
      double v = std::max<double>(lastVelocity, 0);
      double delta_x = (curve->getPathLength() - distanceTraveled);

      return -std::min<double>(
          pow(v, 2) /
              (2 * delta_x),
          curveProfile->getMaxAcceleration());
   }
}

bool HomingPath::completed(double x, double y)
{
   return (hypot(x - endpoint.at(0), y - endpoint.at(1)) <= distanceTolerance) || lastVelocity < 0;
}