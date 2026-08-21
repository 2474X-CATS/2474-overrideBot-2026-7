#include "odometry.h" 
#include "../utilities/functools.h" 

double Odometry::INERTIAL_WHEEL_RADIUS = 25.4; 

double Odometry::GOAL_WIDTH = 6 * 25.4;

Location* Odometry::locations[13] = { 
   new Location( 
     "all_nat_neu", 
     TILE_SIZE_MM * 4, TILE_SIZE_MM, 
     Odometry::GOAL_WIDTH/2
   ),  
   new Location(
     "all_nat_all", 
     TILE_SIZE_MM * 2, TILE_SIZE_MM *1, 
     Odometry::GOAL_WIDTH/2
   ),  
   new Location( 
     "all_for_all", 
     TILE_SIZE_MM *1, TILE_SIZE_MM *2, 
     Odometry::GOAL_WIDTH/2
   ),  
   new Location(
     "all_for_neu", 
     TILE_SIZE_MM *1, TILE_SIZE_MM *4,
     Odometry::GOAL_WIDTH/2
   ),   
   new Location( 
     "opp_nat_neu", 
     TILE_SIZE_MM *5, TILE_SIZE_MM *2, 
     Odometry::GOAL_WIDTH/2
   ),  
   new Location(
     "opp_nat_all", 
     TILE_SIZE_MM *5, TILE_SIZE_MM *4, 
     Odometry::GOAL_WIDTH/2
   ),   
   new Location( 
     "opp_for_all", 
     TILE_SIZE_MM *4, TILE_SIZE_MM *5, 
     Odometry::GOAL_WIDTH/2
   ),   
   new Location( 
     "opp_for_neu", 
     TILE_SIZE_MM *2, TILE_SIZE_MM *5, 
     Odometry::GOAL_WIDTH/2
   ),  
   new Location(
     "central_goal", 
     TILE_SIZE_MM *3, TILE_SIZE_MM *3, 
     Odometry::GOAL_WIDTH/2
   ),  
   new Location( 
     "matchloader_bottom_left",
     TILE_SIZE_MM *0.5, TILE_SIZE_MM *0.5, 
     TILE_SIZE_MM / 2
   ),  
   new Location(
     "matchloader_bottom_right", 
     TILE_SIZE_MM *5.5, TILE_SIZE_MM *0.5, 
     TILE_SIZE_MM / 2
   ),   
   new Location( 
     "matchloader_top_left", 
     TILE_SIZE_MM *0.5, TILE_SIZE_MM *5.5,
     TILE_SIZE_MM / 2
   ),   
   new Location(
     "matchloader_top_right", 
     TILE_SIZE_MM *5.5, TILE_SIZE_MM *5.5,
     TILE_SIZE_MM / 2
   )
};

Location* Odometry::getLocation(int index){ 
   return locations[index];
}

void Odometry::init(){  
   calibratePerspective();
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


} 

void Odometry::setStartingOdometry(){ //Not finished
  double halfWidth = (ROBOT_WIDTH_MM/2); 
  double halfLength = (ROBOT_LENGTH_MM/2);  

  double cornerX = TILE_SIZE_MM; 
  double cornerY = TILE_SIZE_MM; 

  double offsetX = halfWidth; 
  double offsetY = halfHeight; 

  double angleHeading = 90; 

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

void Odometry::calibratePerspective(){ 
  if (!get<bool>("starting_left")){ 
     return;
  }  
  for (int i = 0; i < 8; i++){ 
    Location* currentLocation = getLocation(i);  
    string name = currentLocation->getName(); 
    bool vertical = (name[0] == 'a' && name[4] = "f") || (name[0] == 'o' && name[4] = "n");
    if (vertical){  
      currentLocation->setY((TILE_SIZE_MM * 6) - currentLocation->getY()); 
    } 
     currentLocation->setX((TILE_SIZE_MM * 6) - currentLocation->getX()); 
  }

}