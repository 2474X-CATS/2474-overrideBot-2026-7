#ifndef __CLAW_H__ 
#define __CLAW_H__ 

#include "../architecture/subsystem.h" 
#include "../architecture/command.h"   

#include "vex.h" 

//vex::PORT5 

class Claw : public Subsystem {  
    
    private:  
       static Claw* globalPtr;  
       static const double MAXIMUM_TOLERABLE_DISTANCE; 
       static const int SCORE_DELAY_MILLIS; 
       static const int PICKUP_DELAY_MILLIS;

       vex::motor roller;

       vex::distance objectDetector;

       int lastTransitionStamp = 0;    

       bool rollingIn = false; 
       bool rollingOut = false; 
       
       bool sensesObject(); 

    public:   
       using Subsystem::get; 
       static Claw& getObject();
       
       Claw();
       
       void init() override; 
       void periodic() override; 
       void updateTelemetry() override; 
       void stop() override; 

       void stateControl();
       
    protected:
       using Subsystem::set;
}; 

class RunClaw : public Command<Claw> { 
  private: 
   Claw& clawRef; 

  public:   

   static CommandInterface* getCommand(){ 
      return new RunClaw(Claw::getObject()); 
   } 
   RunClaw(Claw& claw) :   
   Command<Claw>(claw),
   clawRef(claw) 
   {};   

  protected: 
   void start() override; 
   void periodic() override; 
   bool isOver() override; 
   void end() override;
};

#endif 