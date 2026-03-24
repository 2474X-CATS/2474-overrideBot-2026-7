#ifndef __BEZIER_H__
#define __BEZIER_H__

#include <array>
using std::array;

class BezierCurve
{
private:
   static int STANDARD_STEPS;

   array<array<double, 2>, 3> points;

   double approximateLength();

   double generateXPos(double t);
   double generateYPos(double t);

   double pathLength;

public:
   BezierCurve(array<array<double, 2>, 3> points);

   array<double, 2> generatePoint(double t);

   void generatePoint(double t, array<double, 2> &arr);

   double getPathLength();
};

#endif
