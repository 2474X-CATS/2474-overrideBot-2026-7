#include "claw.h" 

Claw* Claw::globalPtr = nullptr; 

const double Claw::MAXIMUM_TOLERABLE_DISTANCE = 75; 
const int Claw::SCORE_DELAY_MILLIS = 150;
const double Claw::MIN_CLEARANCE = 100;

Claw::Claw() : 
Subsystem( 
   "claw", 
   {   
    (EntrySet){"active", EntryType::BOOL}, 
    (EntrySet){"in_possession", EntryType::BOOL},
    (EntrySet){"waiting", EntryType::BOOL}, 
    (EntrySet){"clamped", EntryType::BOOL}, 
    (EntrySet){"auto_clamp_enabled", EntryType::BOOL}
   } 
),
clamp(vex::pneumatics(Brain.ThreeWirePort.A)),
objectDetector(vex::distance(vex::PORT12))
{
   globalPtr = this; 
};

Claw& Claw::getObject(){ 
    return *globalPtr;
}

void Claw::init(){  
    clamp.set(false); 
    clearance = MIN_CLEARANCE;
}

void Claw::periodic(){  
    clamp.set(!get<bool>("clamped"));
} 

void Claw::stop(){
   clamp.set(false); 
}

bool Claw::sensesObject(){
    return objectDetector.objectDistance(vex::distanceUnits::mm) < MAXIMUM_TOLERABLE_DISTANCE;
}

void Claw::updateDrivercontrol(){ 
   set<bool>("clamped", RobotState::getStateOf("clamp")); 
} 

void Claw::updateAuto(){
   SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));
   switch (pos){ 
      case MATCHLOADING:
        set<bool>("clamped", get<bool>("in_possession") && get<bool>("auto_clamp_enabled"));
        break;
      case PRIMED: 
        set<bool>("clamped", true); 
        break; 
      case AUTO:
        if (get<bool>("waiting")){
          if (Brain.Timer.time() - lastTransitionStamp >= SCORE_DELAY_MILLIS){ 
            set<bool>("active", false);
            set<bool>("waiting", false);
            Telemetry::inst.placeValueAt<bool>(true, "ss_manager", "task_completed"); 
            clearance = 0;
          } 
        } else if (get<bool>("active")){ 
           set<bool>("waiting", true);
           set<bool>("clamped", false);
           lastTransitionStamp = Brain.Timer.time();
        }
        break;
   }
}

void Claw::updateTelemetry(){ 
   set<bool>("in_possession", sensesObject());  
   if (RobotState::getStateOf("in_autonomous")){ 
      updateAuto();
   } else { 
      updateDrivercontrol();
   }
}  

void RunClaw::start(){ 
  return;
}

void RunClaw::periodic(){ 
   clawRef.periodic();
} 

bool RunClaw::isOver(){ 
    return false;
} 

void RunClaw::end(){ 
    return;
} 

//------------------------------------------------

