#include "../architecture/graphics.h" 
#include <array> 

class ColorPicker : public Sprite {  
   private:
     bool isBlue = true;
   public:  
     
     bool getIsBlue(); 

     ColorPicker(int x, int y): 
                   Sprite(x, y, 120, 25){};   
     
     void draw() override;  
     void update() override; 
     void mousePressed(int mx, int my) override; 
     void mouseReleased() override;
};

class SidePicker : public Sprite { 
    private: 
      bool isLeft = true; 
    public:  
      
      bool getIsLeft();  

      SidePicker(int x, int y): 
         Sprite(x, y, 120, 25){};   
    
      void draw() override;  
      void update() override; 
      void mousePressed(int mx, int my) override; 
      void mouseReleased() override;
        
};


