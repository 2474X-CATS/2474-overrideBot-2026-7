#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#include "architecture/command.h"  

#include "subsystems/claw.h"
#include "subsystems/drivebase.h" 
#include "subsystems/elevator.h" 
#include "subsystems/forearm.h" 


class RunSuperStructure : public ParallelCommandGroup {  

    public:

      static CommandInterface* getCommand(){ 
        return new RunSuperStructure(); 
      }

      RunSuperStructure(): 
      ParallelCommandGroup(RunElevator::getCommand()) 
      { 
         chainAnd(RunForearm::getCommand())-> 
         chainAnd(RunClaw::getCommand());
      }
};  

class ModifyRobotState : public Command<DummySystem> {  
    
    private:
      std::string modDirectory; 
      std::string modName; 
      bool entryVal; 

    public:
      
      static CommandInterface* getCommand(std::string dir, std::string nm, bool val){ 
         return new ModifyRobotState(GLOBAL_DUMMY, dir, nm, val);
      }

      ModifyRobotState(DummySystem& dummy, std::string dir, std::string nm, bool val): 
      Command<DummySystem>(dummy),
      modDirectory(dir),
      modName(nm),
      entryVal(val)
      {};
    
    protected:
      void start() override;
      void periodic() override; 
      bool isOver() override;
      void end() override;
};


class WaitFor : public Command<DummySystem>{   

    private: 
      int startingTimestamp;   
      int duration;

    public:

      static CommandInterface* getCommand(double durationMillis){ 
         return new WaitFor(GLOBAL_DUMMY, durationMillis); 
      }

      WaitFor(DummySystem& dummy, double durationMillis):  
      Command<DummySystem>(dummy),  
      duration(durationMillis){};
    
    protected:
      void start() override; 
      void periodic() override; 
      bool isOver() override;  
      void end() override;
}; 


class WaitUntil : public Command<DummySystem> { 

    private:  

       std::string directory; 
       std::string name; 
       bool desiredBool;

    public:

       static CommandInterface* getCommand(std::string dir, std::string nm, bool desiredBool){ 
         return new WaitUntil(GLOBAL_DUMMY, dir, nm, desiredBool);
       }
       
       WaitUntil(DummySystem& dummy, std::string dir, std::string nm, bool db): 
       Command<DummySystem>(dummy),   
       directory(dir),
       name(nm), 
       desiredBool(db)
       {}; 

    protected:    
      void start() override; 
      void periodic() override; 
      bool isOver() override;  
      void end() override;
       
};





#endif