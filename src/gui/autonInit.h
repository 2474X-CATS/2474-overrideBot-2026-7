#include "graphics.h" 
#include <array> 


typedef struct { 
  std::string name; 
  std::string description; 
  int index; 
  bool hasLeftSide; 
  bool hasRightSide; 
} AutonOption; 
 

class ColorPicker : public Sprite {  
   private:
     bool isBlue = true;
   public:  
     
     bool getIsBlue(); 

     ColorPicker(int x, int y): 
                   Sprite(x, y, 100, 25){};   
     
     void draw() override;  
     void update() override; 
     void mousePressed(int mx, int my) override; 
     void mouseReleased() override;
}; 

class ExitBlock : public Sprite {  

    public: 
      ExitBlock(int x, int y) : 
      Sprite(x,y,200, 50){}; 

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
         Sprite(x, y, 100, 25){};   
    
      void draw() override;  
      void update() override; 
      void mousePressed(int mx, int my) override; 
      void mouseReleased() override;
        
};  

class LoadingScreen : public Sprite {  
    private: 
      double angle; 
    public:  
      LoadingScreen(): 
         Sprite(190, 70, 100, 100){};   
    
      void draw() override;  
      void update() override; 
      void mousePressed(int mx, int my) override; 
      void mouseReleased() override;
};  

//----------------------------------------------------  

class RoutineCatalog : public Sprite { 
   private:  

     int chosenIndex; 
     std::vector<AutonOption> autos;  
    
   public: 
     RoutineCatalog(int x, int y, std::vector<AutonOption> autosShell); 

     void draw() override; 
     void update() override; 
     void mousePressed(int mx, int my) override;   
     void mouseReleased() override;   

     void renderCell(AutonOption option, int yPos); 

     AutonOption getCurrentAuto();  
     int getAutonIndex();
};
 

class AutonDisplay : public Sprite {   
    private:  
      RoutineCatalog* catalog;   
      SidePicker* sidePicker;
    public: 
      AutonDisplay(int x, int y, RoutineCatalog* catalog, SidePicker* sidePicker); 
      
      void draw() override; 
      void update() override; 
      void mousePressed(int mx, int my) override;  
      void mouseReleased() override;  
      
};


