#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#include "architecture/command.h"  

#include "subsystems/claw.h"
#include "subsystems/drivebase.h" 
#include "subsystems/elevator.h" 
#include "subsystems/forearm.h" 


class RunSuperStructure : public ParallelCommandGroup {  
    //private: 

      //std::string exitDirectory; 
      //std::string exitEntry;

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

    //protected: 
      //bool isOver() override;
};  

class Wait : public Command<DummySystem> {  

    public: 
       Wait(DummySystem dummySystem): 
       Command<DummySystem>(dummySystem)
       {}

    protected:  
       void start() override;
       void periodic() override; 
       bool isOver() override; 
       void end() override;
};  



#endif