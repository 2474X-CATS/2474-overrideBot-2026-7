#ifndef __LOCATION_H__
#define __LOCATION_H__

#include "../architecture/telemetry.h"
#include "../utilities/functools.h"


class Location
{
public:
  Location(string name, double centX, double centY, double zoneRad) : centerX(centX), centerY(centY), zoneRadius(zoneRad), locationName(name) {};

  string getName();
  array<double, 2> getProjectedSetpoint(double distFrom, double theta);

  bool isRobotFacing(double robotX, double robotY, double robotHeading);
  bool isRobotClose(double robotX, double robotY); 
  
  double getY();
  double getX(); 
  double getRadius(); 

  void setX(double x); 
  void setY(double y); 
  void setRadius(double radius);

private:

  double centerX;
  double centerY;
  double zoneRadius;

  string locationName;
};

#endif