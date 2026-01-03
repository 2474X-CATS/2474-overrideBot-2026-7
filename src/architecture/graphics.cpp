#include "graphics.h"

void drawRectangle(int x, int y, int width, int height, uint32_t color){ 
    Brain.Screen.drawRectangle(x, y, width, height, color);
}; 

void drawEllipse(int x , int y, int radius, uint32_t color){ 
    Brain.Screen.drawCircle(x, y, radius, color);
};  

void drawLine(int x1, int y1, int x2, int y2, uint32_t color){  
    Brain.Screen.setPenColor(color);
    Brain.Screen.drawLine(x1, y1, x2, y2); 
};

void renderText(char* str, int x, int y, uint32_t color){  
    Brain.Screen.setPenColor(color);
    Brain.Screen.printAt(x, y, str);
};   

int getMouseX(){ 
    return Brain.Screen.xPosition();
} 

int getMouseY(){ 
    return Brain.Screen.yPosition(); 
}

bool Sprite::running = true; 
std::vector<Sprite*> Sprite::allSprites; 
vex::color Sprite::globalColor; 
uint32_t Sprite::BACKGROUND_COLOR = Sprite::globalColor.white; 


void Sprite::init()
{
  Brain.Screen.pressed(globalMousePressed);
  Brain.Screen.released(globalMouseReleased); 
}

void Sprite::refreshBackground()
{
  Brain.Screen.clearScreen(BACKGROUND_COLOR);
}

void Sprite::redraw()
{
  for (Sprite *sprite : allSprites)
  {
    sprite->draw();
  }
}

void Sprite::refreshSpriteLogic()
{
  for (Sprite *sprite : allSprites)
  {
    sprite->update();
  }
}

void Sprite::globalMousePressed()
{
  int mx = Brain.Screen.xPosition();
  int my = Brain.Screen.yPosition();
  for (Sprite *sprite : allSprites)
  {
    if ((mx >= sprite->x && mx <= sprite->x + sprite->width) && (my >= sprite->y && my <= sprite->y + sprite->height))
    {
      sprite->pressed = true;
      sprite->mousePressed(mx, my);
      break;
    };
  }
}

void Sprite::globalMouseReleased()
{
  for (Sprite *sprite : allSprites)
  {
    if (sprite->pressed)
    {
      sprite->pressed = false;
      sprite->mouseReleased();
      break;
    };
  }
}

void Sprite::filterDead()
{
  for (int i = allSprites.size() - 1; i >= 0; i--)
  {
    if (allSprites[i]->dead)
    {
      allSprites.erase(allSprites.begin() + i);
    }
  }
} 

void Sprite::frameLoop(){ 
   init();
   while (running){
        refreshBackground();  
        redraw();  
        refreshSpriteLogic();  
        filterDead();
        vex::wait(30, vex::msec);  
       }
}


void ColoredBlock::draw(){ 
  drawRectangle(x,y,width,height, color);
} 

void ColoredBlock::update(){ 
  return;
} 

void ColoredBlock::mousePressed(int mx, int my){ 
  color = globalColor.blue;
} 

void ColoredBlock::mouseReleased(){ 
  color = globalColor.cyan;
};