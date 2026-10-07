#include "intake.h" 

Intake* Intake::globalPtr = nullptr;

Intake& Intake::getObject(){ 
    return *globalPtr;
}  

Intake::Intake(): 
    Subsystem( 
      "intake", 
      { 
         (EntrySet){"intaking", EntryType::BOOL}, 
         (EntrySet){"outtaking", EntryType::BOOL}
      }
    ),  
    intakeMotor(vex::motor(vex::PORT9))
    { 
     globalPtr = this;
    };

void Intake::init(){ 
    return;
} 

void Intake::periodic(){ 
    if (get<bool>("intaking")){ 
       intakeMotor.spin(vex::directionType::fwd, 12, vex::voltageUnits::volt);
    } else if (get<bool>("outtaking")){ 
       intakeMotor.spin(vex::directionType::rev, 12, vex::voltageUnits::volt);
    } else {
       intakeMotor.stop();
    }
} 

void Intake::stop(){ 
    intakeMotor.stop();
} 

void Intake::updateTelemetry(){ 
    SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position")); 
    bool still = Telemetry::inst.getValueAt<bool>("ss_manager", "setpoints_reached"); 
    if (still){  
        if (pos == SuperStructurePosition::GROUND){ 
          set<bool>("intaking", true);  
          set<bool>("outtaking", false);
        } else if (RobotState::getStateOf("outtaking") && pos == SuperStructurePosition::PRIMED){ 
          set<bool>("intaking", false);
          set<bool>("outtaking", true);
        } else { 
          set<bool>("intaking", false);
          set<bool>("outtaking", false);
        }
    } else {  
        if (RobotState::getStateOf("override")){ 
          set<bool>("intaking", false); 
          set<bool>("outtaking", true); 
        } else { 
          set<bool>("intaking", false); 
          set<bool>("outtaking", false);
        }
    } 
} 

void RunIntake::start(){ 
  return;
} 

void RunIntake::periodic(){ 
  intakeRef.periodic();
} 

bool RunIntake::isOver(){ 
  return false;
} 

void RunIntake::end(){ 
  return;
}
