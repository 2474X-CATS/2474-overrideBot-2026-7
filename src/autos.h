#ifndef __AUTOS_H__
#define __AUTOS_H__

#include "commands.h"
#include <vector>
using std::vector;

vector<CommandInterface *> closed_side_left();
vector<CommandInterface *> closed_side_right();

vector<CommandInterface *> renegade_left();
vector<CommandInterface *> renegade_right();

vector<CommandInterface *> center_cleanup_left();
vector<CommandInterface *> center_cleanup_right();

vector<CommandInterface *> control_rush_left();
vector<CommandInterface *> control_rush_right();

vector<CommandInterface *> switcheroo_left();

vector<CommandInterface *> empty();
vector<CommandInterface *> do_nothing();
vector<CommandInterface *> drive_forward();

#endif