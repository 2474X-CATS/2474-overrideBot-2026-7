#include "claw.h" 

Claw* Claw::globalPtr = nullptr; 

const double Claw::MAXIMUM_TOLERABLE_DISTANCE = 50; 
const int Claw::SCORE_DELAY_MILLIS = 250; 
const int Claw::PICKUP_DELAY_MILLIS = 450;

Claw::Claw() : 
Subsystem( 
   "claw", 
   { 
    (EntrySet){"active", EntryType::BOOL}, 
    (EntrySet){"in_possession", EntryType::BOOL},
    (EntrySet){"waiting", EntryType::BOOL}
   } 
),  
clamp(vex::pneumatics(Brain.ThreeWirePort.A)),
objectDetector(vex::distance(vex::PORT5))
{ 
   globalPtr = this;
}; 

Claw& Claw::getObject(){ 
    return *globalPtr;
}

void Claw::init(){ 
    return;
}

void Claw::periodic(){   
    clamp.set(clenched); 
}

bool Claw::sensesObject(){  
    return objectDetector.objectDistance(vex::distanceUnits::mm) < MAXIMUM_TOLERABLE_DISTANCE;
}

void Claw::updateTelemetry(){
   set<bool>("in_possession", sensesObject()); 
   stateControl();
}  

void Claw::stop(){  
    return;
} 


void Claw::stateControl(){  
   SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));   
   
   bool still = Telemetry::inst.getValueAt<bool>("ss_manager", "setpoints_reached");
   
   if (get<bool>("waiting")){
      if (get<bool>("active") && (Brain.Timer.time() - lastTransitionStamp >= SCORE_DELAY_MILLIS)){ 
        set<bool>("active", false);
        Telemetry::inst.placeValueAt<bool>(true, "forearm", "active");
        set<bool>("waiting", false);
      } else if ((Brain.Timer.time() - lastTransitionStamp >= PICKUP_DELAY_MILLIS)){ 
        set<bool>("waiting", false);
      }  
   } else if (pos == SuperStructurePosition::AUTO){ //Whenever macro is running
      if (get<bool>("active")){
         set<bool>("waiting", true);
         clenched = false;
         lastTransitionStamp = Brain.Timer.time();
      }
   } else {
      if (!still){
        clenched = true;
      } else {  
        switch (pos){ 
          case STANDING:
             clenched = false;
             if (get<bool>("in_possession")){  
                if (RobotState::getStateOf("in_autonomous")){ 
                 set<bool>("waiting", true);
                }
                clenched = true;
                lastTransitionStamp = Brain.Timer.time();
             }
             break; 
          case GROUND:
             clenched = false;
             break;
          case PRIMED:    
             clenched = true;
             break;  
          default: 
             break;
        } 
      } 
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

