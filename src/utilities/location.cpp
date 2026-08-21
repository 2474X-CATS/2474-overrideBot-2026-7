#include "location.h"
#include "../architecture/robotConfig.h"


double Location::getX()
{
   return centerX;
};

double Location::getY()
{
   return centerY;
};

double Location::getRadius()
{
   return zoneRadius;
} 

void Location::setX(double x){ 
   centerX = x;
} 

void Location::setY(double y){ 
   centerY = y;
}
 
void Location::setRadius(double radius){ 
   zoneRadius = radius;
}

bool Location::isRobotFacing(double robotX, double robotY, double robotHeading)
{ 
   double dist = hypot(centerX - robotX, centerY - robotY);
   double projectedX = robotX + (cos(toRadians(robotHeading)) * dist);
   double projectedY = robotY + (sin(toRadians(robotHeading)) * dist);
   return hypot(centerX - projectedX, centerY - projectedY) <= zoneRadius;
}


bool Location::isRobotClose(double robotX, double robotY)
{
   return hypot(centerX - robotX, centerY - robotY) <= zoneRadius + (hypot(ROBOT_LENGTH_MM, ROBOT_WIDTH_MM) / 2);
}

array<double, 2> Location::getProjectedSetpoint(double distFrom, double theta)
{
   array<double, 2> res;
   res[0] = centerX + cos(toRadians(theta)) * distFrom;
   res[1] = centerY + sin(toRadians(theta)) * distFrom;
   return res;
}


string Location::getName()
{
   return locationName;
}
