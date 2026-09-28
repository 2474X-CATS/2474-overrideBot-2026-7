#ifndef __PATH_BOARD_H__ 
#define __PATH_BOARD_H__ 

#include "graphics.h" 
#include "../control/path.h"  

class PathBoard : public Sprite {  
     
    
    public:  
      PathBoard(Trajectory& traj, int steps): 
      Sprite(120,0,240,240),
      trajectory(traj)
      { 
        trajectory.generatePoints(points, steps);
      };

    private:
      Trajectory& trajectory;
      std::vector<Point> points;  

      int normalizeX(double nx); 
      int normalizeY(double ny); 

    protected:  
      void draw() override;    
      void update() override {};  

      void mousePressed(int mx, int my) override {};   
      void mouseReleased() override {}; 
};


#endif