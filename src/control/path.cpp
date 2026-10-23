#include "path.h" 
#include "../utilities/functools.h" 
#include "../architecture/robotConfig.h" 

void Trajectory::generatePoints(std::vector<PathFrame>& resultantVector, int steps){  
   double nInterval = 1.0 / steps;
   double currentN = 0;
   while (currentN <= 1){
     PathFrame p;
     p.pt.x = generateX(currentN);
     p.pt.y = generateY(currentN);
     p.heading = generateTheta(currentN); 
     p.velocity = generateSpeed(currentN);
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

double Line::calculateLength(){ 
   return hypot(pStart.x - pEnd.x, pStart.y - pEnd.y); 
} 

double Line::generateX(double n){
   return pStart.x + cos(toRadians(heading)) * calculateLength() * n;
}

double Line::generateY(double n){ 
   return pStart.y + sin(toRadians(heading)) * calculateLength() * n;
}  

double Line::generateSpeed(double n){ 
   if (n < nAccelCap){
     return n / nAccelCap;
   } else if (n > 1 - nAccelCap){
     return (1.0 - n) / nAccelCap;
   } else {
     return 1;
   }
}  

double Line::generateTheta(double n){ 
   return heading;
}

//---------------------------------------------------------------------------- 

Arc::Arc(Point start, Point end, double initialHeading):
  Trajectory(),
  startPoint(start),
  endPoint(end)
  {   
   double supposedAngle = angleSum(toDegrees(atan2(end.y - start.y, end.x - start.x)), 0);  
   thetaTrans = angleDifference(supposedAngle, initialHeading);
   curveDirection = static_cast<int>(copysign(1,thetaTrans)) * -1; 
  };

double Arc::calculateLength(){
   return toRadians(fabs(thetaTrans)) * getRadius();
} 

double Arc::generateX(double n){
   Point center = getCenter();
   double rad = getRadius();
   double projectionAngle = thetaTrans * n; //(calculateLength() * 1.0 * n * curveDirection * -1) / rad; 
   double originHeading = angleSum(toDegrees(atan2(startPoint.y - center.y, startPoint.x - center.x)), 0);
   return center.x + cos(toRadians(projectionAngle) + toRadians(originHeading)) * rad;
}

double Arc::generateY(double n){
   Point center = getCenter();
   double rad = getRadius();
   double projectionAngle = thetaTrans * n;//(calculateLength() * 1.0 * n * curveDirection * -1) / rad;
   double originHeading = angleSum(toDegrees(atan2(startPoint.y - center.y, startPoint.x - center.x)), 0); 
   return center.y + sin(toRadians(projectionAngle) + toRadians(originHeading)) * rad; 
}

double Arc::generateTheta(double n){   
   Point center = getCenter();
   double originHeading = getOriginHeading();
   double projectionAngle = (calculateLength() * 1.0 * n * curveDirection * -1) / getRadius(); 
   return angleSum(toDegrees(atan2(startPoint.y - center.y, startPoint.x - center.x)), curveDirection * -1 * projectionAngle);
} 

double Arc::generateSpeed(double n){ 
   double scaleMax = (1 - fabs(thetaTrans) / 180.0);
   if (n < nAccelCap){ 
     return (n / nAccelCap) * scaleMax;
   } else if (n > 1 - nAccelCap){ 
     return ((1.0 - n) / nAccelCap) * scaleMax;
   } else {
     return scaleMax;
   }
}

double Arc::getOriginHeading(){
   Point center = getCenter();
   return angleSum(toDegrees(atan2(startPoint.y - center.y, startPoint.x - center.x)), 0); 
}

double Arc::getRadius(){
   double dist = hypot(startPoint.x - endPoint.x, startPoint.y - endPoint.y);
   return fabs(dist / (2 * sin(toRadians(thetaTrans / 2.0))));
} 

Point Arc::getCenter(){
   Point center;
   double d = hypot(startPoint.x - endPoint.x, startPoint.y - endPoint.y); 
   center.x = ((startPoint.x + endPoint.x) / 2) + curveDirection * ((endPoint.y - startPoint.y)/d) * sqrt(pow(getRadius(), 2) - (pow(d,2) / 4));
   center.y = ((startPoint.y + endPoint.y) / 2) + curveDirection * ((startPoint.x - endPoint.x)/d) * sqrt(pow(getRadius(), 2) - (pow(d,2) / 4));
   return center;
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

double Bezier::generateX(double n){ 
   return (1 - n)*((1 - n)*pStart.x + n*pHandle.x)+n*((1 - n) * pHandle.x + n * pEnd.x);
} 

double Bezier::generateY(double n){
   return (1 - n)*((1 - n)*pStart.y + n*pHandle.y)+n*((1 - n) * pHandle.y + n * pEnd.y);
}

double Bezier::generateSpeed(double n){    
   double res; 

   double px = generateX(n); 
   double py = generateY(n); 

   double pythagDist = hypot(pHandle.y - pStart.y, pHandle.x - pStart.x) + hypot(pHandle.y - pEnd.y, pHandle.x - pEnd.x); 
   double secantDist = hypot(pStart.y - pEnd.y, pStart.x - pEnd.x);

   double maxScale = 1.0 - (secantDist / pythagDist);   
   
   double secantX = (pStart.x + pHandle.x + pEnd.x) / 3.0;
   double secantY = ((pEnd.y - pStart.y) / (pEnd.x - pStart.x)) * (secantX - pStart.x) + pStart.y;  
   
   double refX = (secantX + pHandle.x) / 2.0;
   double refY = (secantY + pHandle.y) / 2.0;  

   double hDist = hypot(px - refX, py - refY); 
   
   double speedScale = 1.0 - (hDist / secantDist); 
   
   double scaledSpeed = std::min<double>(1.0 - (speedScale * maxScale), 1);  
   
   /*
   if (n < nAccelCap){ 
     res = (n / nAccelCap);
   } else if (n > 1 - nAccelCap){ 
     res = ((1.0 - n) / nAccelCap);
   } else {
     res = 1;
   }  
   */
   return scaledSpeed; //std::min<double>(res, scaledSpeed); 
}

double Bezier::generateTheta(double n){  
   double nStep = 1/500.0;  
   Point backPoint; 
   Point frontPoint; 
   if (n < nStep){
      backPoint.x = generateX(0); 
      backPoint.y = generateY(0); 
      frontPoint.x = generateX(nStep); 
      frontPoint.y = generateY(nStep); 
   } else if (n > 1 - nStep){
      frontPoint.x = generateX(1); 
      frontPoint.y = generateY(1); 
      backPoint.x = generateX(1-nStep); 
      backPoint.y = generateY(1-nStep);
   } else {
      frontPoint.x = generateX(n + nStep/2.0);
      frontPoint.y = generateY(n + nStep/2.0);
      backPoint.x = generateX(n - nStep/2.0);
      backPoint.y = generateY(n - nStep/2.0);
   }
   return angleSum(toDegrees(atan2(frontPoint.y - backPoint.y, frontPoint.x - backPoint.x)), 0);
}
















