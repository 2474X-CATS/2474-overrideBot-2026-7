#ifndef __TOGGLE_H__ 
#define __TOGGLE_H__ 

#include "../architecture/subsystem.h"
#include "../architecture/command.h"

class Toggle : public Subsystem { 

    private:    
      static Toggle* globalPtr; 

      vex::motor toggle;
      vex::optical colorSensor; 

    public:  

      static Toggle& getObject(); 

      Toggle();
      
      void init() override; 
      void periodic() override; 
      void updateTelemetry() override; 
      void stop() override;
}; 



class RunToggle : public Command<Toggle> {  

  private:  
  
   Toggle& toggleRef; 

  public:   

   static CommandInterface* getCommand(){ 
      return new RunToggle(Toggle::getObject()); 
   } 

   RunToggle(Toggle& toggle) :   
   Command<Toggle>(toggle),
   toggleRef(toggle)
   {};   

  protected:  

   void start() override; 
   void periodic() override; 
   bool isOver() override; 
   void end() override;
};


#endif 