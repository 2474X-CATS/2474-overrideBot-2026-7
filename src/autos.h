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


vector<CommandInterface *> closed_side_left();
vector<CommandInterface *> closed_side_right(); 

vector<CommandInterface*> auto_skills();

vector<CommandInterface *> renegade_left();
vector<CommandInterface *> renegade_right();

vector<CommandInterface *> center_cleanup_left();
vector<CommandInterface *> center_cleanup_right();

vector<CommandInterface *> control_rush_left();
vector<CommandInterface *> control_rush_right();

vector<CommandInterface *> switcheroo_left(); 
vector<CommandInterface*> switcheroo_right(); 

vector<CommandInterface *> empty();
vector<CommandInterface *> do_nothing();
vector<CommandInterface *> drive_forward(); 



#endif