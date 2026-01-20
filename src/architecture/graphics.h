#ifndef __GRAPHICS__H__ 
#define __GRAPHICS__H__ 

#include "vex.h"   
#include <vector>  

#include <string> 
using std::string;

extern void drawRectangle(int x, int y, int width, int height, uint32_t color);

extern void drawEllipse(int x , int y, int radius, uint32_t color);

extern void drawLine(int x1, int y1, int x2, int y2, uint32_t color);

extern void renderText(string text, int x, int y, uint32_t penColor, uint32_t highlightColor, vex::fontType font);

extern int getMouseX();

extern int getMouseY(); 

extern void drawLogo(bool isColorBlue);

class Sprite {   

    protected: 
      int x; 
      int y; 
      int width; 
      int height;   

      bool pressed = false;   
      bool dead = false;  

      static void init(); 

      static void refreshBackground();

      static void redraw();
      
      static void refreshSpriteLogic();

      static void globalMousePressed();

      static void globalMouseReleased();

      static void filterDead();
      
      static std::vector<Sprite*> allSprites;   

      static uint32_t BACKGROUND_COLOR; 

      static bool running; 

    public:  
       
      static vex::color globalColor;   

      static void frameLoop();
      
      Sprite(int x, int y, int width, int height): 
      x(x), 
      y(y), 
      width(width), 
      height(height)
      { 
        allSprites.push_back(this);
      }; 

      virtual void draw() = 0;    
      virtual void update() = 0;  

      virtual void mousePressed(int mx, int my){};   
      virtual void mouseReleased(){};  

};  


class ColoredBlock : public Sprite {  
  private: 
    uint32_t color = globalColor.cyan; 
  public: 
    ColoredBlock(int x, int y, int width, int height) : 
    Sprite(x,y,width,height){}; 

    void draw() override;  

    void update() override; 

    void mousePressed(int mx, int my) override; 

    void mouseReleased() override;
}; 




#endif 