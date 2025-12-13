#include "autoConfig.h"
#include "vex.h"

#include <vector>
using std::vector;

class Sprite
{
private:
   static vector<Sprite *> allSprites;
   
   static vex::color Color; 
   
   static void escapeMethod(); 

   static int32_t getMouseX()
   {
      return Brain.Screen.xPosition();
   }

   static int32_t getMouseY()
   {
      return Brain.Screen.yPosition();
   };

   static void runGlobalMousePressed()
   {
      int32_t mouseX = getMouseX();
      int32_t mouseY = getMouseY();
      for (Sprite *sprite : allSprites)
      {
         if ((mouseX >= sprite->x && mouseX <= sprite->x + sprite->width) && (mouseY >= sprite->y && mouseY <= sprite->y + sprite->height))
         {
            sprite->mousePressed(mouseX, mouseY);
            sprite->pressing = true;
            return;
         }
      }
   }

   static void runGlobalMouseReleased()
   {
      for (Sprite *sprite : allSprites)
      {
         if (sprite.pressing)
         { 
            sprite->mouseReleased();
            sprite->pressing = false;
            return;
         }
      }
   } 

protected: 
   static void drawRectangle(int x, int y, int width, int height, uint32_t blockColor)
   {
      Brain.Screen.setFillColor(blockColor);
      Brain.Screen.drawRectangle(x, y, width, height);
   }

   static void drawCircle(int x, int y, int radius, uint32_t blockColor)
   {
      Brain.Screen.setFillColor(blockColor);
      Brain.Screen.drawCircle(x, y, radius);
   }

   static void renderText(string message, int x, int y, vex::fontType font, uint32_t color)
   {
      Brain.Screen.setFont(font);
      Brain.Screen.setPenColor(color);
      Brain.Screen.printAt(x, y, message.c_str());
   }

   int x;
   int y;
   int width;
   int height;

   bool pressing = false;

public:
   static bool running;


   static void initialize()
   {
      Brain.Screen.pressed(runGlobalMousePressed); 
      Brain.Screen.released(runGlobalMouseReleased);  
      frameLoop(); 
   } 

   static void disable(){ 
      Brain.Screen.pressed(escapeMethod); 
      Brain.Screen.released(escapeMethod);  
      running = false; 
   }

   static void frameLoop()
   {
      while (running)
      {
         for (Sprite *sprite : allSprites)
         {
            sprite->draw();
         }
         for (Sprite *sprite : allSprites)
         {
            sprite->update();
         }
         vex::wait(20, vex::msec);
      }
   };

   Sprite(int x, int y, int width, int height) : x(x),
                                                 y(y),
                                                 width(width),
                                                 height(height)
   {
      allSprites.push_back(this);
   };

   virtual void draw() { return; };
   virtual void update() { return; };
   virtual void mousePressed(int32_t mouseX, int32_t mouseY) { return; };
   virtual void mouseReleased() { return; };
};

vector<Sprite *> Sprite::allSprites; 

bool Sprite::running = true;  

//--------------------------------------------------------------------------

typedef enum { 
   LEFT_CORNER, 
   RIGHT_CORNER, 
   FACE_LEFT, 
   FACE_RIGHT
} StartingPos; 

struct typedef { 
  bool isBlue; 
  StartingPos startingPosition;  
} InitConstants; 

class ColorChooser : protected Sprite {  
   private: 
     int32_t currentColor;   
   public: 
     ColorChooser() : 
     Sprite(50,50,100, 50)
     { 
       currentColor = Sprite::Color.blue;
     };   
     
     bool isBlue(){ 
      return currentColor = Sprite::Color.blue; 
     }; 

     void 

     
}

//---------------------------------------------------------------------------

InitConstants initializeMatch(){ 
  
}; 
