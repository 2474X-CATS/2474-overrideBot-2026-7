#include "bezier.h"
#include "math.h"

int BezierCurve::STANDARD_STEPS = 500;

BezierCurve::BezierCurve(array<array<double, 2>, 3> points)
{
  this->points = points;
  this->pathLength = this->approximateLength();
}

double BezierCurve::generateXPos(double t)
{
  return (pow(1 - t, 2) * this->points[0][0]) + (2 * (1 - t) * t * this->points[1][0]) + (pow(t, 2) * this->points[2][0]);
}

double BezierCurve::generateYPos(double t)
{
  return (pow(1 - t, 2) * this->points[0][1]) + (2 * (1 - t) * t * this->points[1][1]) + (pow(t, 2) * this->points[2][1]);
}

array<double, 2> BezierCurve::generatePoint(double t)
{
  array<double, 2> arr;
  t = std::min<double>(std::max<double>(t, 0), 1); // Clamping between 0 and 1
  arr[0] = generateXPos(t);
  arr[1] = generateYPos(t);
  return arr;
};

void BezierCurve::generatePoint(double t, array<double, 2> &arr)
{
  arr[0] = generateXPos(t);
  arr[1] = generateYPos(t);
}

double BezierCurve::approximateLength()
{
  double accumulativeDist = 0;
  double t_interval = 1.0 / STANDARD_STEPS;
  double currentT = t_interval;
  array<double, 2> lastPos = generatePoint(0);
  array<double, 2> currentPos;
  while (currentT <= 1)
  {
    currentPos[0] = generateXPos(currentT);
    currentPos[1] = generateYPos(currentT);
    accumulativeDist += sqrt(pow(currentPos[0] - lastPos[0], 2) + pow(currentPos[1] - lastPos[1], 2));
    lastPos[0] = currentPos[0];
    lastPos[1] = currentPos[1];
    currentT += t_interval;
  }
  return accumulativeDist;
}

double BezierCurve::getPathLength()
{
  return this->pathLength;
}