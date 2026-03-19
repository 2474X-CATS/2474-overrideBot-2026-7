#ifndef __PATH_H__
#define __PATH_H__

#include "../utilities/bezier.h"
#include "trapezoidalMotion.h"
#include "pidcontroller.h"
#include "math.h"

typedef struct
{
   double linearVelocity;  // The calculated base speed derived from a trapzeoidal motion profile
   double angularVelocity; // Based on the calculated linear velocity and the angular error calculate the angular velocity
} PathFrameOutput;

typedef struct
{
   array<double, 2> point;
   double progressT; // Closest t point
} BezierReferencePoint;

typedef struct
{
   PIDConstants pidConstants;
   TrapezoidConstants motionConstants;
   double maximumCentripetalAcceleration;
   double positionX;
   double positionY;
   double angleHeading;
} PathMetadata;

class HomingPath
{

private:
   static double TUNED_LDIST;
   static double TUNED_L_SCALE;
   static double OPTIMUM_TOLERANCE;

   BezierCurve *curve = nullptr;
   pidcontroller *turnController = nullptr;
   TrapezoidalMotionProfile *curveProfile = nullptr;

   double lDist;
   double maxCentripetalAcceleration;
   double lastVelocity = 0;

   double lastTimestamp;

   double pathProgress = 0;
   double deriveMaxVelocity(double radius);

   double distanceTolerance = 0;
   double distanceTraveled = 0;

   double k_scale;

   array<double, 2> endpoint;

   void updateAnchorPoint(double closestT);

   double getOptimumLookaheadDist(double velocity);

   TrapezoidalMotionProfile *getProfile();

   BezierReferencePoint findReferencePoint(double x, double y, double lookAhead);

   double getAcceleration();

   double getPathProgress();

   BezierCurve *getCurve();

public:
   HomingPath(BezierCurve *curve, TrapezoidConstants motionConstants, PIDConstants pidconstants, double lookAheadDistance, double k_scale, double maxCentripAccel, double distTolerance);

   HomingPath(array<array<double, 2>, 3> points, TrapezoidConstants motionConstants, PIDConstants pidconstants, double maxCentripAccel);

   HomingPath(array<array<double, 2>, 2> points, PathMetadata metadata);

   PathFrameOutput calculateFrameOutput(double x, double y, double heading, double timestamp);

   void init(double timestamp);

   bool completed(double x, double y);
};

class CirclePath
{

private:
   bool activated = false;

   double startingVelocity = 0;
   double endingVelocity = 0;

   double initialHeading;

   pidcontroller *turnController = nullptr;
   TrapezoidalMotionProfile *profile = nullptr;

   double radius;
   double lastTimestamp = -1;
   double lastOmega = 0;

   double arcLength;

   bool straight = false;
   int turningDirection = 0;
   int drivingDirection = 1;

   bool cuttingCorners;
   array<double, 2> endpoint;

   double projectedHeading;

   double endingHeading;

   double getAngularVelocity(double linearVelocity, double heading, double timestamp);

   void setStartingVelocity(double velocity);
   void setEndingVelocity(double velocity);

   double getMaximumVelocity();

   double getEndpointX();
   double getEndpointY();
   double getEndingHeading();

public:
   CirclePath(array<double, 2> endPoint, bool cuttingCorners, PathMetadata metadata);
   CirclePath(array<double, 2> endPoint, bool cuttingCorners);

   PathFrameOutput calculateFrameOutput(double x, double y, double heading, double timestamp);

   static void linkLeftToRight(CirclePath *path1, CirclePath *path2);

   void transformMetadata(PathMetadata *metadata);

   void activate(PathMetadata metadata);

   void init(double timestamp);

   bool completed(double timestamp);
};

#endif