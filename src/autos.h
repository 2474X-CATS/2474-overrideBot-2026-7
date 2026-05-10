#ifndef __AUTOS_H__
#define __AUTOS_H__

#include "commands.h" 

#include <vector>  
#include <string> 
#include <array>

using std::vector;  
using std::string;  
using std::array;


typedef struct { 
  string name; 
  string description; 
  int index; 
  bool hasLeftSide; 
  bool hasRightSide; 
} AutonOption; 


typedef struct
{
  string name;
  string desc;
  array<vector<CommandInterface*>, 2> autos;
} Routine;  


AutonOption convertRoutineToOption(Routine routine, int index);  
vector<AutonOption> getOptionVector(vector<Routine> routines);
vector<Routine> generateRoutinePool(); 
 
//------------------------------------------------------------------------------- 


#endif