#include "arm.h" 


void Arm::init(){ 
   return;
} 

void Arm::periodic(){ 
   leftMotor.setVelocity(-50, vex::velocityUnits::pct); 
   rightMotor.setVelocity(50, vex::velocityUnits::pct); 

   leftMotor.spin(vex::directionType::rev);
   rightMotor.spin(vex::directionType::rev); 

} 

void Arm::updateTelemetry(){ 
    return;
} 

void Arm::stop(){ 
    leftMotor.stop(); 
    rightMotor.stop(); 
}