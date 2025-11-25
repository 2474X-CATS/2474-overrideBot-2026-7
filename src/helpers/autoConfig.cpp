#include "autoConfig.h"  
#include "vex.h"  

#include <vector>
using std::vector;  


class Sprite { 
    private:   

       static vector<Sprite*> allSprites;  
       
    protected:  

       static void drawRectangle(int x, int y, int width, int height, uint32_t blockColor){  
          Brain.Screen.setFillColor(blockColor);
          Brain.Screen.drawRectangle(x, y, width, height);
       } 

       static void drawCircle(int x, int y, int radius, uint32_t blockColor){  
          Brain.Screen.setFillColor(blockColor);
          Brain.Screen.drawCircle(x, y, radius);
       }  
       
       static void disable(){ 
          running = false;
       }; 

       int x; 
       int y; 
       int width; 
       int height;   

       

    public:   
       
       static bool running; 

       static void frameLoop(){   
         while (running){   
            for (Sprite* sprite : allSprites){ 
               sprite->draw(); 
            } 
            for (Sprite* sprite : allSprites){ 
               sprite->update();
            }
            vex::wait(20, vex::msec); 
             
        }
       };

       static vex::color Color; 

       Sprite(int x, int y, int width, int height) : 
       x(x), 
       y(y), 
       width(width), 
       height(height)
       { 
        allSprites.push_back(this);
       }; 

       virtual void draw(){return;}; 
       virtual void update(){return;};  
};  



class ColoredBlock : protected Sprite {  
   
   private:
     uint32_t blockColor;  

   public: 
     ColoredBlock(int x, int y, int width, int height, uint32_t blockColor) : 
     Sprite(x, y, width, height), 
     blockColor(blockColor)
     {};  
      
     void draw() override { 
        drawRectangle(x,y,width,height, blockColor);
     } 

     void update() override { 
        return;
     }

}; 



vector<Sprite*> Sprite::allSprites;   

bool Sprite::running = true; 

ColoredBlock block = ColoredBlock(20, 150, 100, 50, Sprite::Color.blue);

void runAutonomousMaker(){ 
    Sprite::frameLoop();
};



