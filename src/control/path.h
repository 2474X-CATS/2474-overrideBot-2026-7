#ifndef __PATH_H__
#define __PATH_H__

#include <vector> 

typedef struct { 
   double x = 0.0;
   double y = 0.0;
} Point;

typedef struct {
   Point pt;
   double heading = -1.0;
   double velocity = -1.0;
} PathFrame; 


class Trajectory { //Maps out the points for a path 
   
   protected: 
     double nAccelCap = 0.1; 

   public: 
    
     Trajectory(){};

     void generatePoints(std::vector<PathFrame>& resultantVector, int steps); //How far apart each point should be when generating 
     
     void setAccelCap(double nCap){nAccelCap = nCap;}; 

     virtual double calculateLength(){return 0;}; //Length of the trajectory
     
     virtual double generateX(double n){return 0;}; //Find x-coordinate
     virtual double generateY(double n){return 0;}; //Find y-coordinate  
     virtual double generateTheta(double n){return 0;};
     virtual double generateSpeed(double n){return 0;}; //Returns a scaled version

}; 

class Bezier : public Trajectory {  

    private:

      Point pStart; 
      Point pHandle;  
      Point pEnd; 

    public:

      Bezier(Point start, Point handle, Point end);

      double calculateLength() override;

      double generateX(double n) override; 
      double generateY(double n) override;   
      double generateTheta(double n) override; 
      double generateSpeed(double n) override; 

};

class Line : public Trajectory {   

    private:

      Point pStart; 
      Point pEnd; 

      double heading;

    public:

      Line(Point start, Point end); 

      double calculateLength() override;
       
      double generateX(double n) override;
      double generateY(double n) override;  
      double generateTheta(double n) override; 
      double generateSpeed(double n) override; 

}; 

class Arc : public Trajectory {   

  private: 

    int curveDirection;

    Point startPoint;
    Point endPoint;

    double thetaTrans;

    Point getCenter();

    double getRadius(); 
    double getOriginHeading(); 

  public:  

    Arc(Point start, Point end, double initialHeading);
    
    double calculateLength() override;

    double generateX(double n) override; 
    double generateY(double n) override; 
    double generateTheta(double n) override;
    double generateSpeed(double n) override;
 
};


#endif