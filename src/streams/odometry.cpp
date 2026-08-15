#include "odometry.h" 
#include "../utilities/functools.h" 

double Odometry::INERTIAL_WHEEL_RADIUS = 25.4; 

void Odometry::init(){ 
   setStartingOdometry();  
   lastTimestamp = Brain.Timer.time(); 
} 

void Odometry::refreshData(){ 
     
    double currentTimestamp = Brain.Timer.time(); 
    double delta = (currentTimestamp - lastTimestamp) / 1000.0;

    double currentVelocity = (rot.velocity(vex::velocityUnits::rpm) / 60) * 2 * M_PI * INERTIAL_WHEEL_RADIUS; 
    double currentHeading = gyro.heading();

    double xPos = get<double>("x_position_mm"); 
    double yPos = get<double>("y_position_mm"); 

    double distance =  currentVelocity * delta;

    if (get<bool>("oriented_c")){ 
       currentHeading = flipOrientation(currentHeading);
    } 

    if (RobotState::getStateOf("inverted")){ 
       distance *= -1;
       currentHeading = angleSum(currentHeading, 180);
    }    
    
    set<double>("heading_deg", currentHeading);
    set<double>("immediate_distance", distance); 
    set<double>("velocity_ms", currentVelocity);

    currentHeading = toRadians(currentHeading); 
    
    xPos += cos(currentHeading) * distance; 
    yPos += sin(currentHeading) * distance; 

    set<double>("x_position_mm", xPos); 
    set<double>("y_position_mm", yPos); 
  
    lastTimestamp = currentTimestamp;  

    Brain.Screen.printAt(20, 100, "Angle Degrees: %.2f", get<double>("heading_deg"));

} 

void Odometry::setStartingOdometry(){ //Not finished
  double halfWidth = (ROBOT_WIDTH_MM/2); 
  double halfLength = (ROBOT_LENGTH_MM/2);  

  double cornerX; 
  double cornerY; 

  double offsetX; 
  double offsetY; 

  double angleHeading; 
  
  /*
  if (RobotState::getStateOf("field_type_is_vex")){ 
     if (get<bool>("starting_left")){ 
      
     } else { 

     }
  } else {  
     if (get<bool>("starting_left")){ 
        
     } else { 

     }
  } 
  */ 

  cornerX = 0; 
  cornerY = 0; 

  offsetX = halfWidth; 
  offsetY = halfLength; 

  angleHeading = 90;  

  // Filler code just to test odom

  if (get<bool>("oriented_c")){ 
     angleHeading = flipOrientation(angleHeading);
  }  
  
  gyro.calibrate(); 
  while (gyro.isCalibrating()){ 
    vex::this_thread::yield();
  } 

  gyro.setHeading(angleHeading, vex::rotationUnits::deg); 
  rot.setReversed(true); 
  
  set<double>("x_position_mm", cornerX + offsetX); 
  set<double>("y_position_mm", cornerY + offsetY); 
  

}