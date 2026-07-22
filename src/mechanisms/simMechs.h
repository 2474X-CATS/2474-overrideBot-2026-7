#ifndef __MECH_H__ 
#define __MECH_H__ 

#include "../gui/graphics.h"  


class ElevatorMech : public Sprite {  
    private:  
      static double CHANNEL_WIDTH; 
      static double DEFAULT_X; 
      static double DEFAULT_Y;  
      static double MAX_WIDTH; 
      static double STAGE_HEIGHT;

      double getHeight(); 
      
    public: 
      ElevatorMech() : 
        Sprite(0,0,0,0) 
        {}; 
      
      void draw() override; 
      void update() override; 
      void mousePressed(int mx, int my) override; 
      void mouseReleased() override; 

      double getPivotX(); 
      double getPivotY();
};  

class ForearmMech : public Sprite {  
    private:  
      ElevatorMech& elevRef;  

      static double FOREARM_HEIGHT;   

      
    public:   
      
      static double FOREARM_LENGTH;  

      double getAngle();  
      double getPivotX(); 
      double getPivotY();  

      ForearmMech(ElevatorMech& elev) : 
        Sprite(0,0,0,0),  
        elevRef(elev)
        {};
      
      void draw() override; 
      void update() override; 
      void mousePressed(int mx, int my) override; 
      void mouseReleased() override;
};

class ClawMech : public Sprite {   

  private: 
    ForearmMech& foreRef;  
    static double CLAW_WIDTH; 
    static double CLAW_HEIGHT; 
     
    double getOutwardAngle();  
    
    double getPivotX(); 
    double getPivotY();

  public: 
    ClawMech(ForearmMech& fore): 
      Sprite(0,0,0,0), 
      foreRef(fore) 
      {};    
    
    void draw() override; 
    void update() override; 
    void mousePressed(int mx, int my) override; 
    void mouseReleased() override; 
}; 

class Console : public Sprite {  
  private: 
     void displayStatus(); 
     void displayActions(); 

     void displayPrimingAction(); 
     void displayClawAction();  
     void displaySwitchPickupMode(); 
     void displayActivateMacro(); 
     void displayEnableGameObject(); 

     bool checkRect(int rectX, int rectY, int rectWidth, int rectHeight);  

     void maintainPrimingAction(); 

     bool checkClawAction(); 
     bool checkSwitchUp(); 
     bool checkActivateMacro(); 
     bool checkGOEnable();

  public:  
     Console() : 
     Sprite(240, 0, 240, 240) 
     {};

     void draw() override; 
     void update() override; 
     void mousePressed(int mx, int my) override; 
     void mouseReleased() override;
};








#endif