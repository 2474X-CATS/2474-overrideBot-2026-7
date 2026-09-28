#ifndef __PATH_H__
#define __PATH_H__

#include <vector> 


typedef struct { 
   double x = 0.0; 
   double y = 0.0; 
   double heading = -1.0; 
} Point; 


class Trajectory { //Maps out the points for a path 

   public: 

     Trajectory(){};

     virtual Point getEndPoint(){ 
      Point p; 
      return p;
     };    

     void generatePoints(std::vector<Point>& resultantVector, int steps); //How far apart each point should be when generating 

     virtual double calculateLength(){return 0;}; //Length of the trajectory
     
     virtual double generateX(double n){return 0;}; //Find x-coordinate
     virtual double generateY(double n){return 0;}; //Find y-coordinate 

}; 

class Bezier : public Trajectory {  

    private:

      Point pStart; 
      Point pHandle;  
      Point pEnd; 

    public:

      Bezier(Point start, Point handle, Point end);

      Point getEndPoint() override;  

      double calculateLength() override;

      double generateX(double n) override; 
      double generateY(double n) override;  

};

class Line : public Trajectory {   

    private:

      Point pStart; 
      Point pEnd; 

      double heading;

    public: 

      Line(Point start, Point end);

      Point getEndPoint() override;  

      double calculateLength() override;
       
      double generateX(double n) override;
      double generateY(double n) override; 

}; 

class Arc : public Trajectory {  
   
  private:
    int curveDirection;

    Point startPoint; 
    Point endPoint; 

    Point getCenter(); 
    double getRadius(); 
    
    double getPointDist(); 

    double getAngleChange(); 

  public: 
    Arc(Point start, Point end);  

    Point getEndPoint() override;  

    double calculateLength() override; 
    double generateX(double n) override; 
    double generateY(double n) override;
 
  
};


#endif