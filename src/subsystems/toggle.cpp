#include "toggle.h" 

Toggle* Toggle::globalPtr = nullptr;

Toggle& Toggle::getObject(){ 
    return *globalPtr;
}  

Toggle::Toggle(): 
    Subsystem(
      "toggle",
      { 
         (EntrySet){"is_on", EntryType::BOOL}, 
         (EntrySet){"must_flip", EntryType::BOOL}
      }
    ),
    toggle(vex::motor(vex::PORT10)),
    colorSensor(vex::optical(vex::PORT8))
    {
     globalPtr = this;
    };

void Toggle::init(){  
    set<bool>("must_flip", true); 
} 

void Toggle::periodic(){  
    bool spinning = RobotState::getStateOf("toggle");  
    if (colorSensor.isNearObject() && !spinning){
      if (RobotState::getStateOf("is_team_color_blue")){ 
         spinning = colorSensor.color() != vex::color::blue; 
      } else { 
         spinning = colorSensor.color() != vex::color::red; 
      } 
      set<bool>("must_flip", spinning);
    } else { 
      set<bool>("must_flip", false);
    }
    
    if (spinning){ 
      toggle.spin(vex::directionType::rev, 7.2, vex::voltageUnits::volt); 
    } else {
      toggle.stop();
    }

}

void Toggle::stop(){ 
    return;
} 

void Toggle::updateTelemetry(){  
    return;
} 

void RunToggle::start(){ 
    return;
} 

void RunToggle::periodic(){ 
  toggleRef.periodic();
} 

bool RunToggle::isOver(){ 
  return false;
} 

void RunToggle::end(){ 
  return;
}
