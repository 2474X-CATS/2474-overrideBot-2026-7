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
         (EntrySet){"must_flip", EntryType::BOOL}, 
         (EntrySet){"auto_spin_enabled", EntryType::BOOL}
      }
    ),
    toggle(vex::motor(vex::PORT10)),
    colorSensor(vex::optical(vex::PORT8))
    {
     globalPtr = this;
    };

void Toggle::init(){   
    toggle.setBrake(vex::brakeType::brake);
    set<bool>("must_flip", true);    

    colorSensor.setLight(vex::ledState::on); 
    colorSensor.setLightPower(100);
} 

void Toggle::periodic(){  
    if (colorSensor.isNearObject()){
      if (RobotState::getStateOf("is_team_color_blue")){ 
         set<bool>("must_flip", colorSensor.color() != vex::color::blue); 
      } else { 
         set<bool>("must_flip", colorSensor.color() != vex::color::red); 
      } 
    } else { 
      set<bool>("must_flip", false);
    }

    if (RobotState::getStateOf("toggle") || (get<bool>("auto_spin_enabled") && get<bool>("must_flip"))){  
      double speed = 8;
      if (!RobotState::getStateOf("toggle")){ 
         speed = 5;
      }
      toggle.spin(vex::directionType::rev, speed, vex::voltageUnits::volt); 
    } else {
      toggle.stop();
    }

}

void Toggle::stop(){ 
    return;
} 

void Toggle::updateTelemetry(){  
    if (!RobotState::getStateOf("in_autonomous")){ 
      set<bool>("auto_spin_enabled", true);
    }
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
