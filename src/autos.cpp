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
  return routines;
}

//-------------------------------------------------------------------------------------- 

vector<CommandInterface*> two_pin_alliance_left(){ 
  return {     
    ParallelCommandGroup::makeGroup(
       SequentialCommandGroup::makeGroup(Toggle::getCommand(2))->  
       chainThen(FaceTarget::getCommand(TILE_SIZE_MM * 2, TILE_SIZE_MM))-> 
       chainThen(ApproachTarget::getCommand(TILE_SIZE_MM*2, TILE_SIZE_MM, 80))-> 
       chainThen(Score())-> 
       chainThen(DriveForward::getCommand(-250))
    )->chainWhile(RunSuperStructure::getCommand()) 
  };
}   

vector<CommandInterface*> two_pin_alliance_right(){ 
  return { 
    ParallelCommandGroup::makeGroup(
       SequentialCommandGroup::makeGroup(DriveForward::getCommand(500))->
       chainThen(TurnToHeading::getCommand(180))->
       chainThen(DriveForward::getCommand(500))
    )->chainWhile(RunSuperStructure::getCommand())
  };
} 

