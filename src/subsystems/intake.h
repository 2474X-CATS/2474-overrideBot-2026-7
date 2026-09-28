#ifndef __INTAKE_H__ 
#define __INTAKE_H__ 

#include "../architecture/subsystem.h"
#include "../architecture/command.h"

class Intake : public Subsystem { 

    private:    
      static Intake* globalPtr;
      vex::motor intakeMotor;
      
    public:  

      static Intake& getObject(); 

      Intake();
      
      void init() override; 
      void periodic() override; 
      void updateTelemetry() override; 
      void stop() override;
}; 



class RunIntake : public Command<Intake> {  

  private: 
   Intake& intakeRef; 

  public:   

   static CommandInterface* getCommand(){ 
      return new RunIntake(Intake::getObject()); 
   } 

   RunIntake(Intake& intake) :   
   Command<Intake>(intake),
   intakeRef(intake)
   {};   

  protected: 
   void start() override; 
   void periodic() override; 
   bool isOver() override; 
   void end() override;
};


#endif 