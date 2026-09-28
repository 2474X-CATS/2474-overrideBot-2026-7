#ifndef __PURSUIT_H__ 
#define __PURSUIT_H__

#include "path.h"

class PurePursuit {  

    private:
      std::vector<Point> checkPoints;

      double lDist; 
      int lastFoundIndex = 0;

      Point checkIntersection(double posX, double posY, int i1, int i2); //Between the line
      
      Point findReferencePoint(double posX, double posY); 

    public:  

      PurePursuit(std::vector<Point> points, double lookAheadDist);
      
      void calculateError(double posX, double posY, double heading, double& linearError, double& angularError);

};


#endif