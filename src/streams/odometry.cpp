#include "odometry.h" 
#include "../utilities/functools.h" 

const double Odometry::INERTIAL_WHEEL_RADIUS = 25.4; 
const double Odometry::ANG_ROT_DIST_FROM_CENTER = 3.369586 * 25.4; //
const double Odometry::GOAL_WIDTH = 6 * 25.4; 
const double Odometry::MATCHLOAD_CLEARANCE_MM = 250;

Location* Odometry::locations[13] = { 
   new Location(
     "all_nat_neu",
     TILE_SIZE_MM * 4, TILE_SIZE_MM, 
     Odometry::GOAL_WIDTH/2
   ),
   new Location(
     "all_nat_all", 
     TILE_SIZE_MM * 2, TILE_SIZE_MM * 1, 
     Odometry::GOAL_WIDTH/2
   ),  
   new Location( 
     "all_for_all", 
     TILE_SIZE_MM *1, TILE_SIZE_MM * 2, 
     Odometry::GOAL_WIDTH/2
   ),  
   new Location(
     "all_for_neu", 
     TILE_SIZE_MM *1, TILE_SIZE_MM * 4,
     Odometry::GOAL_WIDTH/2
   ),   
   new Location( 
     "opp_nat_neu", 
     TILE_SIZE_MM *5, TILE_SIZE_MM * 2, 
     Odometry::GOAL_WIDTH/2
   ),  
   new Location(
     "opp_nat_all", 
     TILE_SIZE_MM *5, TILE_SIZE_MM * 4, 
     Odometry::GOAL_WIDTH/2
   ),   
   new Location( 
     "opp_for_all", 
     TILE_SIZE_MM *4, TILE_SIZE_MM * 5, 
     Odometry::GOAL_WIDTH/2
   ),   
   new Location( 
     "opp_for_neu", 
     TILE_SIZE_MM * 2, TILE_SIZE_MM * 5, 
     Odometry::GOAL_WIDTH/2
   ),  
   new Location(
     "central_goal", 
     TILE_SIZE_MM * 3, TILE_SIZE_MM * 3, 
     Odometry::GOAL_WIDTH/2
   ),  
   new Location( 
     "matchloader_bottom_left",
     TILE_SIZE_MM * 0.5, TILE_SIZE_MM * 0.5, 
     TILE_SIZE_MM / 2
   ),  
   new Location(
     "matchloader_bottom_right", 
     TILE_SIZE_MM * 5.5, TILE_SIZE_MM * 0.5, 
     TILE_SIZE_MM / 2
   ),   
   new Location( 
     "matchloader_top_left", 
     TILE_SIZE_MM * 0.5, TILE_SIZE_MM * 5.5,
     TILE_SIZE_MM / 2
   ),   
   new Location(
     "matchloader_top_right", 
     TILE_SIZE_MM * 5.5, TILE_SIZE_MM * 5.5,
     TILE_SIZE_MM / 2
   )
};

Location* Odometry::getLocation(int index){ 
   return locations[index];
}

Odometry::Odometry() : 
    DataStream( 
      "odometry",
       {  
           (EntrySet){"starting_left", EntryType::BOOL}, 
           (EntrySet){"x_position_mm", EntryType::DOUBLE}, 
           (EntrySet){"y_position_mm", EntryType::DOUBLE}, 
           (EntrySet){"heading_deg", EntryType::DOUBLE}, 
           (EntrySet){"velocity_ms", EntryType::DOUBLE},  
           (EntrySet){"immediate_distance", EntryType::DOUBLE},
           (EntrySet){"oriented_c", EntryType::BOOL},
           (EntrySet){"forward_acceleration", EntryType::DOUBLE}, 
           (EntrySet){"obstructed", EntryType::BOOL}
       }
    ),
    gyro(vex::inertial(vex::PORT16)), 
    linRot(vex::rotation(vex::PORT8)),
    angRot(vex::rotation(vex::PORT13)),
    frontDist(vex::distance(vex::PORT6)),
    leftDist(vex::distance(vex::PORT21)),
    rightDist(vex::distance(vex::PORT15))
    {};

void Odometry::init(){
   set<bool>("oriented_c", true);
   calibratePerspective();
   setStartingOdometry();
   lastTimestamp = Brain.Timer.time();
} 

double Odometry::getHeading(){ 
  double heading = gyro.heading(); 
  if (get<bool>("oriented_c")){ 
       heading = flipOrientation(heading); 
  }  
  return heading;
} 

void Odometry::refreshData(){ 
    
    double currentTimestamp = Brain.Timer.time(); 
    double delta = (currentTimestamp - lastTimestamp) / 1000.0;
    
    double currentHeading = getHeading();
    double omega = gyro.gyroRate(vex::axisType::zaxis, vex::velocityUnits::dps);   

    set<double>("forward_acceleration", gyro.acceleration(vex::axisType::yaxis));
    set<bool>("obstructed", frontDist.objectDistance(vex::distanceUnits::mm) < 50); 

    if (get<bool>("oriented_c")){
       omega *= -1;
    }

    double omegaToRPS = ((omega / 360.0) * (2 * ANG_ROT_DIST_FROM_CENTER * M_PI)) / (2 * M_PI * INERTIAL_WHEEL_RADIUS);

    double posYVelocity = (linRot.velocity(vex::velocityUnits::rpm) / 60) * 2 * M_PI * INERTIAL_WHEEL_RADIUS; 
    double posXVelocity = ((angRot.velocity(vex::velocityUnits::rpm) / 60) - omegaToRPS) * 2 * M_PI * INERTIAL_WHEEL_RADIUS;  

    double posYDistance = posYVelocity * delta;  
    double posXDistance = posXVelocity * delta;

    if (RobotState::getStateOf("inverted")){ 
       posYDistance *= -1;
       posXDistance *= -1;
       currentHeading = angleSum(currentHeading, 180);
    }    
    
    set<double>("heading_deg", currentHeading);
    set<double>("immediate_distance", posYDistance); 
    set<double>("velocity_ms", posYVelocity); 

    currentHeading = toRadians(currentHeading);  

    double xPos = get<double>("x_position_mm"); 
    double yPos = get<double>("y_position_mm"); 
    
    xPos += cos(currentHeading) * posYDistance; 
    yPos += sin(currentHeading) * posYDistance;  

    xPos += sin(currentHeading) * posXDistance; 
    yPos += cos(currentHeading) * posXDistance;

    set<double>("x_position_mm", xPos); 
    set<double>("y_position_mm", yPos);   
    
    /*
    double distX = -1;
    double distY = -1;
    laserCalibrate(distX, distY);
    Brain.Screen.clearLine(120); 
    Brain.Screen.clearLine(140);
    Brain.Screen.printAt(20, 120, "Distance sensor odom: %.2f, %.2f", distX, distY); 
    Brain.Screen.printAt(20, 120, "Angle: %.2f", get<double>("heading_deg")); 
    */
    lastTimestamp = currentTimestamp;

} 


void Odometry::laserCalibrate(double& resX, double& resY){ 
  double xAccum = 0;  
  int xCount = 0;
  double yAccum = 0;   
  int yCount = 0;
  findTranslations(xAccum, yAccum, xCount, yCount, frontDist, 0, ROBOT_LENGTH_MM/2); 
  findTranslations(xAccum, yAccum, xCount, yCount, leftDist, 90, ROBOT_WIDTH_MM/2.0); 
  findTranslations(xAccum, yAccum, xCount, yCount, rightDist, -90, ROBOT_WIDTH_MM/2);  
  if (xCount > 0){ 
    resX = xAccum * 1.0 / xCount;
  }  
  if (yCount > 0){ 
    resY = yAccum * 1.0 / yCount;
  }
  
}

void Odometry::findTranslations(double& xCoords, double& yCoords, int& xCount, int& yCount, vex::distance sensor, double angleOffset, double distanceOffset){ 
   double distance = sensor.objectDistance(vex::distanceUnits::mm) + distanceOffset;  
   double sensorHeading = angleSum(getHeading(), angleOffset);
   double wallHeading;
    
   if (sensorHeading > 45 && sensorHeading < 135){ 
     wallHeading = 90;
   } else if (sensorHeading > 135 && sensorHeading < 225) { 
     wallHeading = 180;
   } else if (sensorHeading > 225 && sensorHeading < 315){ 
     wallHeading = 270;
   } else { 
     wallHeading = 0;
   }
   
   sensorHeading = angleDifference(sensorHeading, wallHeading);    
   double straightDist = fabs(cos(toRadians(sensorHeading)) * distance);   

   if (wallHeading == 90){
    yCoords += TILE_SIZE_MM * 6 - straightDist; 
    yCount++;
   } else if (wallHeading == 270){ 
    yCoords += straightDist;  
    yCount++;
   } else if (wallHeading == 0){ 
    xCoords += TILE_SIZE_MM * 6 - straightDist; 
    xCount++;
   } else { 
    xCoords += straightDist; 
    xCount++;
   }

}


void Odometry::setStartingOdometry(){ //Not finished
  double halfWidth = (ROBOT_WIDTH_MM/2); 
  double halfLength = (ROBOT_LENGTH_MM/2);  

  double cornerX = TILE_SIZE_MM * 3 - halfWidth; 
  double cornerY = 4 * 25.4; 

  double angleHeading = 90; 

  if (get<bool>("oriented_c")){ 
     angleHeading = flipOrientation(angleHeading);
  }
  
  gyro.calibrate();
  while (gyro.isCalibrating()){ 
    vex::this_thread::yield();
  } 

  gyro.setHeading(angleHeading, vex::rotationUnits::deg); 
  linRot.setReversed(true); 
  
  set<double>("x_position_mm", cornerX + halfWidth); 
  set<double>("y_position_mm", cornerY + halfLength); 
  
} 

void Odometry::calibratePerspective(){ 
  if (!get<bool>("starting_left")){
     return;
  }
  for (int i = 0; i < 8; i++){ 
    Location* currentLocation = getLocation(i);  
    string name = currentLocation->getName(); 
    bool vertical = (name[0] == 'a' && name[4] == 'f') || (name[0] == 'o' && name[4] == 'n');
    if (vertical){  
      currentLocation->setY((TILE_SIZE_MM * 6) - currentLocation->getY()); 
    } 
     currentLocation->setX((TILE_SIZE_MM * 6) - currentLocation->getX()); 
  }
}