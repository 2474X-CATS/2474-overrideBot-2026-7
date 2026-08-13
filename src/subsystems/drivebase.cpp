#include "drivebase.h" 

Drivebase* Drivebase::globalPtr = nullptr; 

void Drivebase::init(){ 

  ffConsts.kA = 0; 
  ffConsts.kV = 0; 
  ffConsts.kS = 0; 
   
  correctionConsts.P = 0; 
  correctionConsts.I = 0; 
  correctionConsts.D = 0; 

  leftMotors.setStopping(vex::brakeType::coast);  
  rightMotors.setStopping(vex::brakeType::coast);
}  

Drivebase& Drivebase::getObject(){ 
    return *globalPtr;
}

void Drivebase::periodic(){ 
  arcadeDrive(RobotState::getAxisState(AxisType::M_LEFT_VERTICAL), RobotState::getAxisState(AxisType::M_RIGHT_HORIZONTAL));
}

void Drivebase::updateTelemetry(){ 
  return;
}

void Drivebase::stop(){ 
   leftMotors.stop(); 
   rightMotors.stop();
} 

void Drivebase::manualDrive(double voltage){ 
    leftMotors.spin(vex::directionType::fwd, voltage, vex::voltageUnits::volt); 
    rightMotors.spin(vex::directionType::rev, voltage, vex::voltageUnits::volt);
} 

void Drivebase::manualSpin(double voltage){ 
    leftMotors.spin(vex::directionType::fwd, -voltage, vex::voltageUnits::volt); 
    rightMotors.spin(vex::directionType::rev, voltage, vex::voltageUnits::volt); 
} 

void Drivebase::arcadeDrive(double speed, double rotation){ 
    leftMotors.spin(vex::directionType::fwd, ((speed + rotation) / 100.0) * 12.0, vex::voltageUnits::volt); 
    rightMotors.spin(vex::directionType::rev, ((speed + rotation) / 100.0) * 12.0, vex::voltageUnits::volt); 
}