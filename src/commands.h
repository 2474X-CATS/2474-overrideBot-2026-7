#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#include "architecture/command.h"  

#include "subsystems/claw.h"
#include "subsystems/drivebase.h" 
#include "subsystems/elevator.h" 
#include "subsystems/toggle.h" 

#include "streams/odometry.h"


class RunSuperStructure : public ParallelCommandGroup {  

    public:

      static CommandInterface* getCommand(){ 
        return new RunSuperStructure(); 
      }

      RunSuperStructure(): 
      ParallelCommandGroup(RunElevator::getCommand()) 
      { 
         chainAnd(RunClaw::getCommand())->chainAnd(RunToggle::getCommand());
      }
};

class ModifyRobotState : public Command<DummySystem> {  
    
    private:  

      std::string modDirectory; 
      std::string modName; 
      bool entryVal;
      bool ran = false;

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

class WaitForSSState : public Command<DummySystem> { 

    private:  

       SuperStructurePosition state; 

    public:

       static CommandInterface* getCommand(SuperStructurePosition state){ 
         return new WaitForSSState(GLOBAL_DUMMY, state);
       }
       
       WaitForSSState(DummySystem& dummy, SuperStructurePosition ssState): 
       Command<DummySystem>(dummy),
       state(ssState)
       {}; 

    protected:
      void start() override; 
      void periodic() override; 
      bool isOver() override;  
      void end() override;

};

class ResetPose : public Command<DummySystem> { 
    private:

       vector<ResetFace> faces; 

    public:
       
       static CommandInterface* getCommand(vector<ResetFace> resetFaces){ 
        CommandInterface* comm = new ResetPose(GLOBAL_DUMMY, resetFaces); 
        return SequentialCommandGroup::makeGroup( 
         { 
            comm, 
            WaitFor::getCommand(100)
         }
        );
       };

       ResetPose(DummySystem& dummy, vector<ResetFace> resetFaces): 
       Command<DummySystem>(dummy),
       faces(resetFaces)
       {}; 

    protected:
      void start() override; 
      void periodic() override; 
      bool isOver() override;  
      void end() override;
};

class ResetFromGoal : public Command<Drivebase> { 
   private: 
      int g;  
      int startTime;
      Drivebase& drivebaseRef;   

      bool switched = false;
   public:  

      static CommandInterface* getCommand(Setpoint goal){ 
        return new ResetFromGoal(Drivebase::getObject(), goal);
      };

      ResetFromGoal(Drivebase& drive, Setpoint goal): 
      Command<Drivebase>(drive), 
      drivebaseRef(drive), 
      g(goal){};  

   protected:  
      void start() override; 
      void periodic() override; 
      bool isOver() override;  
      void end() override;
};

CommandInterface* Score();   
CommandInterface* WaitForToggle(); 
CommandInterface* DriveIntoElement(CommandInterface* driveComm);
CommandInterface* DriveIntoGoal(CommandInterface* driveComm);

CommandInterface* SnapHeading();  
CommandInterface* ResetFromGoal(Setpoint goal); 

CommandInterface* Clench(); 

vector<CommandInterface*> RoutineWrap(vector<CommandInterface*> content); 



#endif