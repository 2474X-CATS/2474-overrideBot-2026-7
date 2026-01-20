#ifndef __LOCATION_H__
#define __LOCATION_H__

#include "../architecture/telemetry.h"

#include <cmath>
using std::atan2;
using std::cos;
using std::fmod;
using std::sin;

#include <string>
using std::string;

#include <vector>
using std::array;
using std::vector;

// IN DEVELOPMENT

class Location
{
public:
  Location(string name, double centX, double centY, double zoneRad, double perfEntranceAngle, double angTolerance) : centerX(centX), centerY(centY), zoneRadius(zoneRad), angleTolerance(angTolerance),
                                                                                                                     perfectEntranceAngle(fmod(90 - perfEntranceAngle + 360, 360)), locationName(name) {};

  string getName();

  bool isRobotVisiting(); // Is the robot facing the locations and is its heading within the angle threshold and if the two are touching

  bool isRobotFacing(double robotX, double robotY, double robotHeading);
  bool isRobotInBounds(double robotX, double robotY);
  bool isRobotClose(double robotX, double robotY);

  double getY();
  double getX();
  double getRadius();
  double getPerfectEntranceAngle();

  array<double, 2> getProjectedSetpoint(double distFrom);
  vector<double> getEuclideanAlignmentPath(double distFrom);
  vector<double> getTaxicabAlignmentPath(double distFrom, bool xFirst);

private:
  static double getAngleDiff(double angle1, double angle2);
  static double degreesToRadians(double angle);
  static double getAngleSum(double angle1, double angle2);
  static double normalizeTrigOutput(double radians);

  double centerX;
  double centerY;
  double zoneRadius;
  double angleTolerance;
  double perfectEntranceAngle;

  string locationName;
};

#endif