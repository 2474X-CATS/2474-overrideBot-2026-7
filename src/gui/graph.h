#ifndef __GRAPH_H__ 
#define __GRAPH_H__ 


#include "graphics.h" 
#include "../architecture/telemetry.h"

typedef struct {  

   std::string directory; 
   std::string name;  

   std::string label;

   double getValue();  
   
   std::string getLabel();

} DataSupplier; //Data that returns a double when referenced on  


typedef struct { 
   double timestamp;
   vector<double> yValues;
} Frame;



class Graph : public Sprite {   
    
    private:  
      
      static double colors[7][3]; 

      vector<Frame> frames; 

      vector<DataSupplier> outputSupplier; 

      std::string header; 
      int charsInTitle;

      double maximumValue = 1000; 
      double minimumValue = -1000;  

      double displayTimeRange; 
      double minimumTimestamp;

      double getLocalizedY(double rawY);  
      double getLocalizedX(double timestamp);
  
      void drawFrame();  
      void drawData(); 
      void drawLegend(); 
      void drawTicks();  
      void drawTitle();


    public:
      Graph(std::string title, vector<DataSupplier> outputs): 
      Sprite(40,25,360,180),
      displayTimeRange(5000),
      minimumTimestamp(Brain.Timer.time()), 
      outputSupplier(outputs), 
      header(title), 
      charsInTitle(title.size())
      {};

      void draw() override; 
      void update() override; 
      void mousePressed(int mx, int my) override;  
      void mouseReleased() override;

};


#endif





