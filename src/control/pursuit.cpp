#include "pursuit.h" 

#include "../utilities/functools.h" 
#include "../architecture/robotConfig.h"

PurePursuit::PurePursuit(std::vector<Point> points, double lookAheadDist): 
checkPoints(points), 
lDist(lookAheadDist)
{} 


Point PurePursuit::checkIntersection(double posX, double posY, int i1, int i2){ //i1 and i2 are within bounds
   Point intersection;  
   intersection.x = -1; 
   intersection.y = -1;
   double x1, x2, y1, y2, D, xDist, yDist, tanDist, discrim, minX, minY, maxX, maxY;  
   
   minX = std::min<double>(checkPoints[i1].x, checkPoints[i2].x);  
   minY = std::min<double>(checkPoints[i1].y, checkPoints[i2].y);   

   maxX = std::max<double>(checkPoints[i1].x, checkPoints[i2].x);  
   maxY = std::max<double>(checkPoints[i1].y, checkPoints[i2].y);

   x1 = checkPoints[i1].x - posX; 
   x2 = checkPoints[i2].x - posX; 
   y1 = checkPoints[i1].y - posY; 
   y2 = checkPoints[i2].y - posY;  
   
   xDist = x2 - x1; 
   yDist = y2 - y1; 
   tanDist = hypot(xDist, yDist); 

   D = x1 * y2 - x2 * y1; 
   discrim = pow(lDist, 2) * pow(tanDist, 2) - pow(D, 2); 
   
   if (discrim >= 0){  
     double resX1, resY1, resX2, resY2;
     resX1 = ((D * yDist + copysign(1, yDist) * xDist * sqrt(discrim)) / pow(tanDist, 2)) + posX; 
     resX2 = ((D * yDist - copysign(1, yDist) * xDist * sqrt(discrim)) / pow(tanDist, 2)) + posX;   
     resY1 = ((-D * xDist + fabs(yDist) * sqrt(discrim)) / pow(tanDist, 2)) + posY; 
     resY2 = ((-D * xDist - fabs(yDist) * sqrt(discrim)) / pow(tanDist, 2)) + posY;  
     
     if (resX1 < minX || resX1 > maxX || resY1 < minY || resY1 > maxY){ 
        resX1 = -1; 
        resY1 = -1;
     }  

     if (resX2 < minX || resX2 > maxX || resY2 < minY || resY2 > maxY){ 
        resX2 = -1; 
        resY2 = -1;
     }  

     bool sol1Valid = resX1 != -1; 
     bool sol2Valid = resY1 != -1;  

     if (sol1Valid || sol2Valid){ 
        if (sol1Valid && sol2Valid){ //Which is closest to the next point
            double dist1 = hypot(resX1 - checkPoints[i2].x, resY1 - checkPoints[i2].y); 
            double dist2 = hypot(resX2 - checkPoints[i2].x, resY2 - checkPoints[i2].y); 
            intersection.x = resX1 ? dist1 < dist2 : resX2;
            intersection.y = resY1 ? dist1 < dist2 : resY2;
        } else if (sol1Valid){ 
            intersection.x = resX1; 
            intersection.y = resY1;
        } else { 
            intersection.x = resX2; 
            intersection.y = resY2;
        }
     }
   } 

   return intersection;

} 

Point PurePursuit::findReferencePoint(double posX, double posY){  
  Point goalPt; 
  //bool foundIntersection = false;
  for (int i = lastFoundIndex; i < checkPoints.size() - 1; i ++){ 
     goalPt = checkIntersection(posX, posY, i, i+1); 
     if (goalPt.x != -1){ 
        if (hypot(goalPt.x - checkPoints[i+1].x, goalPt.y - checkPoints[i+1].y) < hypot(posX - checkPoints[i+1].x, posY - checkPoints[i+1].y)){ 
          lastFoundIndex = i; 
          break;
        } else { 
          lastFoundIndex = i + 1;
        }
     } else { 
        goalPt = checkPoints[lastFoundIndex];  
        break;
     }
  }
  return goalPt;
} 

void PurePursuit::calculateError(double posX, double posY, double heading, double& linearError, double& angularError){ 
   Point reference = findReferencePoint(posX, posY); 
   double dist = hypot(posX - reference.x, posY - reference.y); 
   double theta = angleSum(toDegrees(atan2(reference.y - posY, reference.x - posX)), 0);   
   if ((reference.x == 0 && reference.y == 0) || ((lastFoundIndex == checkPoints.size() - 1) && dist < 50)){  
      linearError = 0;
      angularError = 0;  
      return;
   }
   linearError = dist; 
   angularError = angleDifference(theta, heading);  
   Brain.Screen.printAt(20, 160, "Next point: (%.2f, %.2f) Progress: (%d/%d)", reference.x, reference.y, lastFoundIndex, checkPoints.size()); 
   Brain.Screen.clearLine(160);
}