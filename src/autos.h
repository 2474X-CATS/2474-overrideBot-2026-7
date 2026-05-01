#ifndef __AUTOS_H__
#define __AUTOS_H__

#include "commands.h"
#include <vector> 

using std::vector;

typedef struct { 
  std::string name; 
  std::string description; 
  int index; 
  bool hasLeftSide; 
  bool hasRightSide; 
} AutonOption; 


typedef struct
{
  string name;
  string desc;
  array<vector<CommandInterface *>, 2> autos;
} Routine;  


AutonOption convertRoutineToOption(Routine routine, int index);  
vector<AutonOption> getOptionVector(vector<Routine> routines);
vector<Routine> generateRoutinePool(); 
 
//------------------------------------------------------------------------------- 


#endif