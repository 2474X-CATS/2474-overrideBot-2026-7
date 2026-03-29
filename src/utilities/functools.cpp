#include "functools.h" 

double toRadians(double angleDeg){ 
  return angleDeg / 180 * M_PI;
} 

double toDegrees(double angleRads){ 
  return angleRads / M_PI * 180; 
} 

double angleSum(double angle1, double angle2){ 
  return fmod(angle1 + angle2 + 360, 360);
} 

double angleDifference(double angle1, double angle2){ 
  double diff = angle1 - angle2; 
  if (diff > 180 || diff < -180){ 
      diff = (360 - fabs(diff)) * -1 * copysign(1, diff); 
  } 
  return diff; 
} 

double flipOrientation(double angle){ 
  return fmod(angle + (90 - angle) * 2 + 360, 360);
} 

double angleBetweenPts(double x1, double y1, double x2, double y2){ 
  return toDegrees(atan2(y2 - y1, x2 - x1) + M_PI); 
}
