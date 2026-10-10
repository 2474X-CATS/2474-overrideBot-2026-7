#include "autos.h"


/* 
-------------------------------------------------------------------------------------------------------------------------------------------------
     _              _______   _____    _____
    / \    |     |     |     |     |   |
   /___\   |     |     |     |     |   -----
  /     \  |_____|     |     |_____|   ____|
----------------------------------------------------------------------------------------------------------------------------------------------------
*/


AutonOption convertRoutineToOption(Routine routine, int index)
{
  AutonOption option;
  option.description = routine.desc;
  option.name = routine.name;
  option.hasLeftSide = !routine.autos.at(0).empty();
  option.hasRightSide = !routine.autos.at(1).empty();
  option.index = index;
  return option;
}

vector<AutonOption> getOptionVector(vector<Routine> routines)
{
  vector<AutonOption> res;
  for (int index = 0; index < routines.size(); index++)
  {
    res.push_back(convertRoutineToOption(routines.at(index), index));
  }
  return res;
}

vector<Routine> generateRoutinePool(){ 
  vector<Routine> routines;
  /* 
  Add routines here
  */ 
  Routine toggPin; 
  toggPin.name = "Toggle -> Preload"; 
  toggPin.desc = "[Self explanatory]";
  toggPin.autos = {
    two_pin_alliance_left(), 
    two_pin_alliance_right()
  };

  Routine togg; 
  togg.name = "Toggle only"; 
  togg.desc = "[Self explanatory]"; 
  togg.autos = { 
    toggle(), 
    toggle()
  }; 

  routines.push_back(toggPin); 
  routines.push_back(togg);
  return routines;
}

//-------------------------------------------------------------------------------------- 

vector<CommandInterface*> two_pin_alliance_right(){ 
  return {     
    ParallelCommandGroup::makeGroup(
       SequentialCommandGroup::makeGroup(Toggle::getCommand(2))->  
       chainThen(FaceTarget::getCommand(TILE_SIZE_MM * 2, TILE_SIZE_MM))-> 
       chainThen(ApproachTarget::getCommand(TILE_SIZE_MM*2, TILE_SIZE_MM, 80))-> 
       chainThen(Score())->
       chainThen(DriveForward::getCommand(-500))-> 
       chainThen(DriveToSetpoint::getCommand(TILE_SIZE_MM*2, TILE_SIZE_MM * 0.5, RouteType::EUCLIDEAN))-> 
       chainThen(FaceTarget::getCommand(TILE_SIZE_MM, TILE_SIZE_MM * 0.5))->
       chainThen(ApproachTarget::getCommand(TILE_SIZE_MM, TILE_SIZE_MM * 0.5))-> 
       chainThen(TurnToHeading::getCommand(0))
    )->chainWhile(RunSuperStructure::getCommand()) 
  };
}

vector<CommandInterface*> two_pin_alliance_left(){ 
  return {     
    ParallelCommandGroup::makeGroup(
       SequentialCommandGroup::makeGroup(Toggle::getCommand(2))->  
       chainThen(FaceTarget::getCommand(TILE_SIZE_MM*4, TILE_SIZE_MM))-> 
       chainThen(ApproachTarget::getCommand(TILE_SIZE_MM*4, TILE_SIZE_MM, 80))-> 
       chainThen(Score())->
       chainThen(DriveForward::getCommand(-500))-> 
       chainThen(DriveToSetpoint::getCommand(TILE_SIZE_MM*2, TILE_SIZE_MM * 0.5, RouteType::EUCLIDEAN))-> 
       chainThen(FaceTarget::getCommand(TILE_SIZE_MM, TILE_SIZE_MM * 0.5))-> 
       chainThen(ApproachTarget::getCommand(TILE_SIZE_MM, TILE_SIZE_MM * 0.5))-> 
       chainThen(TurnToHeading::getCommand(0))
    )->chainWhile(RunSuperStructure::getCommand()) 
  };
}  

vector<CommandInterface*> toggle(){ 
  return {
    Toggle::getCommand(2),
    DriveForward::getCommand(250),
    ParallelCommandGroup::makeGroup(WaitFor::getCommand(5000))->
    chainWhile(RunSuperStructure::getCommand())
  };
}  

vector<CommandInterface*> auto_skills(){ 
  return {
    ParallelCommandGroup::makeGroup(
       SequentialCommandGroup::makeGroup(DriveForward::getCommand(TILE_SIZE_MM * 2))-> 
       //chainThen(DriveToSetpoint::getCommand(TILE_SIZE_MM * 3, TILE_SIZE_MM * 3 - ROBOT_LENGTH_MM/2, RouteType::EUCLIDEAN))->
       chainThen(Score())
    )->chainWhile(RunSuperStructure::getCommand()) 
  }; 
}


