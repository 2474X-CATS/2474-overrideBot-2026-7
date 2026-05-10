#include "command.h"
#include <stdexcept>
#include <bits/stdc++.h>

//-----------------------------------------------------------------------

ParallelCommandGroup::ParallelCommandGroup(CommandInterface* comm){ 
    chainAnd(comm);
} 

ParallelCommandGroup* ParallelCommandGroup::chainAnd(CommandInterface* comm){ 
    bool isCommandCompatible = integrateSubsystems(comm->getSystems()); 
    if (!isCommandCompatible){ 
        return nullptr; // Calling a function on a nullptr will break everything
    } else {
        participatingCommands.push_back(comm); 
        qualifiers.push_back(comm);
    } 
    return this;
}  

ParallelCommandGroup* ParallelCommandGroup::chainWhile(CommandInterface* comm){ 
    bool isCommandCompatible = integrateSubsystems(comm->getSystems()); 
    if (!isCommandCompatible){ 
        return nullptr; // Calling a function on a nullptr will cause an error
    } else {
        participatingCommands.push_back(comm); 
    } 
    return this;
}    

bool ParallelCommandGroup::integrateSubsystems(vector<std::reference_wrapper<Subsystem>> subsystems){ 
    bool successful = true;
    for (Subsystem& system : subsystems){ 
        for (Subsystem& cachedSystem : allSubsystems){ 
            if (&cachedSystem == &system){ 
                successful = false; 
                break;
            }
        } 
        if (successful){ 
            allSubsystems.push_back(system);
        } else { 
            break;
        }
    } 
    return successful;
} 

void ParallelCommandGroup::start(){ 
    for (CommandInterface* comm : participatingCommands){ 
        comm->start();
    }
} 

void ParallelCommandGroup::periodic(){
    for (int idx = 0; idx < participatingCommands.size(); idx ++){  
        CommandInterface* comm = participatingCommands.at(idx);  
        if (comm == nullptr){ 
            continue;
        } 
        if (comm->isOver()){ 
            comm->end(); 
            participatingCommands[idx] = nullptr;
        } else { 
            comm->periodic(); 
        }
        
    }
} 

void ParallelCommandGroup::end(){ 
    for (CommandInterface* comm : participatingCommands){ 
        if (comm != nullptr){ 
            comm->end();
        }
    }
} 

bool ParallelCommandGroup::isOver(){  
  bool finished = true;
  for (CommandInterface* comm : qualifiers){ 
    if (!comm->isOver()){ 
      finished = false; 
      break;
    }
  } 
  return finished;
} 

ParallelCommandGroup* ParallelCommandGroup::makeGroup(CommandInterface* initialCommand){ 
  return new ParallelCommandGroup(initialCommand);
}

//---------------------------------------------------------------------------------- 

SequentialCommandGroup::SequentialCommandGroup(CommandInterface* initialCommand){ 
    chainThen(initialCommand);
} 

void SequentialCommandGroup::initializeNext(){ 
    commandIndex++; 
    if (commandIndex < numCommands){ 
        commands[commandIndex]->start();
    } 
}

void SequentialCommandGroup::start(){ 
    initializeNext();
} 

void SequentialCommandGroup::periodic(){ 
    if (commands[commandIndex]->isOver()){  
        commands[commandIndex]->end();
        initializeNext();
    } else { 
        commands[commandIndex]->periodic();
    }
} 

bool SequentialCommandGroup::isOver(){ 
    return commandIndex == numCommands - 1;
} 

void SequentialCommandGroup::end(){ 
    return;
} 

SequentialCommandGroup* SequentialCommandGroup::chainThen(CommandInterface* comm){ 
    numCommands++; 
    commands.push_back(comm); 
    return this;
} 

SequentialCommandGroup* SequentialCommandGroup::makeGroup(CommandInterface* comm){ 
    return new SequentialCommandGroup(comm);
}