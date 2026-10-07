#include "claw.h" 

Claw* Claw::globalPtr = nullptr; 

const double Claw::MAXIMUM_TOLERABLE_DISTANCE = 80; 
const int Claw::SCORE_DELAY_MILLIS = 350; 
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
roller(vex::motor(vex::PORT1)),
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
    if (rollingIn){ 
       roller.spin(vex::directionType::fwd, 12, vex::voltageUnits::volt);
    } else if (rollingOut){ 
       roller.spin(vex::directionType::rev, 12, vex::voltageUnits::volt); 
    } else { 
       roller.stop(); 
    }
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
        set<bool>("waiting", false);
        Telemetry::inst.placeValueAt<bool>(true, "forearm", "active"); 
        rollingOut = false;
      } else if ((Brain.Timer.time() - lastTransitionStamp >= PICKUP_DELAY_MILLIS)){ 
        set<bool>("waiting", false);
      }  
   } else if (pos == SuperStructurePosition::AUTO){ //Whenever macro is running
      if (get<bool>("active")){
         set<bool>("waiting", true);
         rollingOut = true; 
         rollingIn = false;
         lastTransitionStamp = Brain.Timer.time();
      }
   } else {
      if (!still){
        rollingOut = false;
        rollingIn = false;
      } else {
        switch (pos){
          case STANDING:
             rollingIn = true;
             rollingOut = false;
             if (get<bool>("in_possession")){
                if (RobotState::getStateOf("in_autonomous")){
                 set<bool>("waiting", true);
                }
                lastTransitionStamp = Brain.Timer.time();
             }
             break; 
          case GROUND:
             rollingIn = true;
             rollingOut = false;
             break;
          case PRIMED: 
             rollingIn = false; 
             rollingOut = RobotState::getStateOf("outtaking");
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

