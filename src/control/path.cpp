#include "path.h" 
#include "../utilities/functools.h" 

void Trajectory::generatePoints(std::vector<Point>& resultantVector, int steps){  
   double nInterval = 1.0 / steps;//lookAheadDist * 1.0 / calculateLength(); 
   double currentN = 0; 
   while (currentN <= 1){   
     Point p; 
     p.x = generateX(currentN); 
     p.y = generateY(currentN);
     resultantVector.push_back(p);
     currentN += nInterval;
   }  
}  

//----------------------------------------------------------------------------- 

Line::Line(Point start, Point end): 
     Trajectory(), 
     pStart(start), 
     pEnd(end){  
     
     heading = toDegrees(atan2(pEnd.y - pStart.y, pEnd.x - pStart.x));  

};

Point Line::getEndPoint()
{
    Point frame;
    frame.x = pEnd.x; 
    frame.y = pEnd.y;
    frame.heading = heading;
    return frame;
}

double Line::calculateLength(){ 
   return hypot(pStart.x - pEnd.x, pStart.y - pEnd.y); 
} 

double Line::generateX(double n){ 
   return pStart.x + cos(toRadians(heading)) * calculateLength() * n;
} 

double Line::generateY(double n){ 
   return pStart.y + sin(toRadians(heading)) * calculateLength() * n; 
} 

//---------------------------------------------------------------------------- 

Arc::Arc(Point start, Point end): //Specify the starting angle not the ending angle
  Trajectory(), 
  startPoint(start)  
  {   
   endPoint.x = end.x; 
   endPoint.y = end.y;
   endPoint.heading = end.heading; 
   curveDirection = static_cast<int>(copysign(1, endPoint.heading - startPoint.heading)) * -1;
  }; 

double Arc::calculateLength(){  
   return toRadians(fabs(getAngleChange())) * getRadius();
} 

Point Arc::getEndPoint(){ 
   return endPoint;
} 

double Arc::getAngleChange(){ 
   return endPoint.heading - startPoint.heading;
} 

double Arc::generateX(double n){  
   Point center = getCenter(); 
   double rad = getRadius();   
   double projectionAngle = (calculateLength() * 1.0 * n * curveDirection * -1) / rad;
   return center.x + cos(projectionAngle + toRadians(center.heading)) * rad;
}

double Arc::generateY(double n){  
   Point center = getCenter();
   double rad = getRadius();
   double projectionAngle = (calculateLength() * 1.0 * n * curveDirection * -1) / rad;
   return center.y + sin(projectionAngle + toRadians(center.heading)) * rad; 
} 

double Arc::getRadius(){ 
   double dist = hypot(startPoint.x - endPoint.x, startPoint.y - endPoint.y); 
   double alpha = toRadians(endPoint.heading - startPoint.heading); 
   return fabs(dist / (2 * sin(alpha / 2)));
} 

Point Arc::getCenter(){  
   Point center;   
   double d = getPointDist(); 
   center.x = ((startPoint.x + endPoint.x) / 2) + curveDirection * ((endPoint.y - startPoint.y)/d) * sqrt(pow(getRadius(), 2) - (pow(d,2) / 4));
   center.y = ((startPoint.y + endPoint.y) / 2) + curveDirection * ((startPoint.x - endPoint.x)/d) * sqrt(pow(getRadius(), 2) - (pow(d,2) / 4));
   center.heading = angleSum(toDegrees(atan2(startPoint.y - center.y, startPoint.x - center.x)), 0); 
   return center;
} 

double Arc::getPointDist(){ 
   return hypot(startPoint.x - endPoint.x, startPoint.y - endPoint.y);
}



//----------------------------------------------------------------------------------


Bezier::Bezier(Point start, Point handle, Point end):  
      Trajectory(),
      pStart(start),
      pHandle(handle),  
      pEnd(end)
      {}; 
   
double Bezier::calculateLength(){  
   double dist = 0;
   for (int i = 1; i <= 500; i++){ 
      dist += hypot(generateX((i-1)/500.0) - generateX(i/500.0), generateY((i-1)/500.0) - generateY(i/500.0));
   } 
   return dist;
   
} 

Point Bezier::getEndPoint(){  
    Point endpoint;  
    double nStep = 1/100.0;
    double finalTheta = angleSum(toDegrees(atan2(pEnd.x - generateX(1.0 - nStep), pEnd.y - generateY(1.0 - nStep))), 0);  
    endpoint.x = pEnd.x; 
    endpoint.y = pEnd.y;
    endpoint.heading = finalTheta; 
    return endpoint; 
} 

double Bezier::generateX(double n){ 
   return (1 - n)*((1 - n)*pStart.x + n*pHandle.x)+n*((1 - n) * pHandle.x + n * pEnd.x);
} 

double Bezier::generateY(double n){
   return (1 - n)*((1 - n)*pStart.y + n*pHandle.y)+n*((1 - n) * pHandle.y + n * pEnd.y);
}




















