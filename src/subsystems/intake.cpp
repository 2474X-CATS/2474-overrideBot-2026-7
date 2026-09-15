#include "intake.h" 

void Intake::init(){ 
    return;
} 

void Intake::periodic(){ 
    if (get<bool>("intaking")){ 
       intakeMotor.spin(vex::directionType::fwd, 12, vex::voltageUnits::volt);
    } else if (get<bool>("outtaking")){ 
       intakeMotor.spin(vex::directionType::fwd, -12, vex::voltageUnits::volt);
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
    if (still && pos == SuperStructurePosition::GROUND){ 
        set<bool>("intaking", RobotState::getStateOf("intaking")); 
        set<bool>("outtaking", RobotState::getStateOf("outtaking")); 
    } else { 
        set<bool>("intaking", false); 
        set<bool>("outtaking", false);
    }
}