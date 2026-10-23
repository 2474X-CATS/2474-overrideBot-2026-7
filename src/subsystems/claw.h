#ifndef __CLAW_H__ 
#define __CLAW_H__ 

#include "../architecture/subsystem.h" 
#include "../architecture/command.h"   

#include "vex.h" 


class Claw : public Subsystem {  
    
    private:  
       static Claw* globalPtr;   

       static const double MAXIMUM_TOLERABLE_DISTANCE; 
       static const int SCORE_DELAY_MILLIS;
       static const double MIN_CLEARANCE; 

       double clearance;  

       vex::pneumatics clamp; 
       vex::distance objectDetector;
       
       int lastTransitionStamp; 

       bool sensesObject();  

       void updateDrivercontrol(); 
       void updateAuto(); 

    public:   
       using Subsystem::get; 
       static Claw& getObject();
       
       Claw();
       
       void init() override; 
       void periodic() override; 
       void updateTelemetry() override; 
       void stop() override; 
       
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