#ifndef __FUNCTOOLS_H__ 
#define __FUNCTOOLS_H__ 

#include "math.h" 

extern double toRadians(double angleDeg); 
extern double toDegrees(double angleRads); 

extern double angleSum(double angle1, double angle2); 
extern double angleDifference(double angle1, double angle2); 
 
extern double flipOrientation(double angle);  

extern double angleBetweenPts(double x1, double y1, double x2, double y2);


#endif