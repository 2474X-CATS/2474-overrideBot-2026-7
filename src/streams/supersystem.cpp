#include "supersystem.h" 

//const double SuperSystem::MINIMUM_SCORING_CLEAREANCE = 100; 

SuperSystem::SuperSystem() :  
    DataStream(  
      "ss_manager", 
      {
          (EntrySet){"macro_requested", EntryType::BOOL},//Do we want to run auto-score?  
          (EntrySet){"position", EntryType::INT}, //The position state the robot is in [AUTO when a macro]
          (EntrySet){"task_completed", EntryType::BOOL}, //Is the macro done running?    
      }
    ){}; 

void SuperSystem::setPosition(int pos){
    set<int>("position", pos);
}

void SuperSystem::init(){ 
    setPosition(SuperStructurePosition::PRIMED);
}

void SuperSystem::refreshData(){

    bool inPossession = Telemetry::inst.getValueAt<bool>("claw", "in_possession"); 

    switch (get<int>("position")){
        case MATCHLOADING:
            if (Telemetry::inst.getValueAt<bool>("claw", "clamped")){
              setPosition(SuperStructurePosition::PRIMED);
            }
            break;
        case PRIMED:
            if (get<bool>("macro_requested")){ 
              Telemetry::inst.placeValueAt<bool>(true, "elevator", "active"); //First subsystem to act
              set<bool>("task_completed", false); //Task is not completed
              setPosition(SuperStructurePosition::AUTO); //Set the position to AUTO (essentially undefined
            } else if (!inPossession){ 
              setPosition(SuperStructurePosition::MATCHLOADING);
            }
            break;
        case AUTO:
            if (get<bool>("task_completed")){ 
              setPosition(SuperStructurePosition::MATCHLOADING);
            }
            break;
        default:
            break;
    }

    set<bool>("macro_requested", false); 
     
}