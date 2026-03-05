#include "path.h" 



PathFrameOutput Path::calculateFrameOutput(double x, double y, double heading){ 
   PathFrameOutput res;  
   BezierReferencePoint refPoint = findReferencePoint(x,y);
    
   double dist = sqrt(pow(x - refPoint[0],2) + pow(y - refPoint[1], 2)); 
   double desiredHeading = (acos((refPoint[0] - x) / dist) + M_PI) / (2 * M_PI) * 360; 
   
   
//array<double,2> referencePoint;///array<double,2> referencePoint;//array<double,2> referencePoint;/array<double,2> referencePoint;
   return res;
} 

BezierReferencePoint Path::findReferencePoint(double x, double y){ 
    BezierReferencePoint res;  
    return res;
}