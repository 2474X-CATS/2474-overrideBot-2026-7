#include "odometry.h" 
#include "../utilities/functools.h" 

const double Odometry::INERTIAL_WHEEL_RADIUS = 25.4; 
const double Odometry::ANG_ROT_DIST_FROM_CENTER = 2.880412 * 25.4; //
const double Odometry::GOAL_WIDTH = 6 * 25.4; 
const double Odometry::MATCHLOAD_CLEARANCE_MM = 250;   

const double Odometry::OFFSET_COR_X = 0;
const double Odometry::OFFSET_COR_Y = 0;

//const Odometry* Odometry::obj = nullptr;

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

Odometry::Odometry(bool usingTendy):
    DataStream(
      "odometry",
       {  
           //(EntrySet){"starting_left", EntryType::BOOL},
           (EntrySet){"x_position_mm", EntryType::DOUBLE},
           (EntrySet){"y_position_mm", EntryType::DOUBLE},
           (EntrySet){"tip_x_position_mm", EntryType::DOUBLE},
           (EntrySet){"tip_y_position_mm", EntryType::DOUBLE},
           (EntrySet){"heading_deg", EntryType::DOUBLE}, 
           (EntrySet){"velocity_ms", EntryType::DOUBLE},  
           (EntrySet){"immediate_distance", EntryType::DOUBLE},
           (EntrySet){"oriented_c", EntryType::BOOL},
           //(EntrySet){"forward_acceleration", EntryType::DOUBLE}, 
           (EntrySet){"obstructed", EntryType::BOOL},  
           (EntrySet){"dr_reset_triggered", EntryType::BOOL},  
           (EntrySet){"ang_reset_triggered", EntryType::BOOL}, 
           (EntrySet){"proxy_reset_triggered", EntryType::BOOL}, 
           (EntrySet){"goal_index", EntryType::INT},
           (EntrySet){"factor_left", EntryType::BOOL},
           (EntrySet){"factor_right", EntryType::BOOL},
           (EntrySet){"factor_front", EntryType::BOOL},
           (EntrySet){"factor_back", EntryType::BOOL},
           (EntrySet){"opened", EntryType::BOOL}
       }
    ),
    gyro(vex::inertial(vex::PORT12)), 
    linRot(vex::rotation(vex::PORT11)),
    angRot(vex::rotation(vex::PORT16)),
    frontDist(vex::distance(vex::PORT7)),
    leftDist(vex::distance(vex::PORT18)),
    rightDist(vex::distance(vex::PORT9)), 
    backDist(vex::distance(vex::PORT21))
    { 
       set<bool>("oriented_c", true); 
    };

Odometry::Odometry():
    DataStream(
      "odometry",
       {
           //(EntrySet){"starting_left", EntryType::BOOL},  
           (EntrySet){"x_position_mm", EntryType::DOUBLE}, 
           (EntrySet){"y_position_mm", EntryType::DOUBLE},  
           (EntrySet){"tip_x_position_mm", EntryType::DOUBLE},
           (EntrySet){"tip_y_position_mm", EntryType::DOUBLE},
           (EntrySet){"heading_deg", EntryType::DOUBLE}, 
           (EntrySet){"velocity_ms", EntryType::DOUBLE},  
           (EntrySet){"immediate_distance", EntryType::DOUBLE},
           (EntrySet){"oriented_c", EntryType::BOOL},
           //(EntrySet){"forward_acceleration", EntryType::DOUBLE}, 
           (EntrySet){"obstructed", EntryType::BOOL},
           (EntrySet){"dr_reset_triggered", EntryType::BOOL},  
           (EntrySet){"ang_reset_triggered", EntryType::BOOL}, 
           (EntrySet){"proxy_reset_triggered", EntryType::BOOL}, 
           (EntrySet){"goal_index", EntryType::INT},
           (EntrySet){"factor_left", EntryType::BOOL}, 
           (EntrySet){"factor_right", EntryType::BOOL}, 
           (EntrySet){"factor_front", EntryType::BOOL}, 
           (EntrySet){"factor_back", EntryType::BOOL},
           (EntrySet){"opened", EntryType::BOOL}
       }
    ),
    gyro(vex::inertial(vex::PORT14)), 
    linRot(vex::rotation(vex::PORT7)),
    angRot(vex::rotation(vex::PORT6)),
    frontDist(vex::distance(vex::PORT5)),
    leftDist(vex::distance(vex::PORT13)),
    rightDist(vex::distance(vex::PORT4)), 
    backDist(vex::distance(vex::PORT21))
    { 
       set<bool>("oriented_c", true); 
    }; 

void Odometry::init(){  
   linRot.setReversed(true);  
   angRot.setReversed(true);  

   gyro.calibrate();
   while (gyro.isCalibrating()){ 
    vex::this_thread::yield();
   }  
} 

double Odometry::getHeading(){ 
  double heading = gyro.heading(); 
  if (get<bool>("oriented_c")){ 
       heading = flipOrientation(heading); 
  }  
  return heading;
}    

double Odometry::getProcessedHeading(){
  double heading = getHeading();  
  if (RobotState::getStateOf("inverted")){ 
        heading = angleSum(heading, 180);
  } 
  return heading; 
}

double Odometry::getOmega(){ 
  double omega = gyro.gyroRate(vex::axisType::zaxis, vex::velocityUnits::dps);    
  if (get<bool>("oriented_c")){
      omega *= -1;
  }   
  return omega; 
}

double Odometry::getStrafeVelocity(){ 
  double linStrafeV = (angRot.velocity(vex::velocityUnits::rpm) / 60) * 2 * M_PI * INERTIAL_WHEEL_RADIUS;  
  double angStrafeV = toRadians(getOmega()) * ANG_ROT_DIST_FROM_CENTER;
  if (fabs(angStrafeV) >= fabs(linStrafeV)){ 
    return 0;
  }  
  double strafeV = linStrafeV - angStrafeV; 
  strafeV = RobotState::getStateOf("inverted") ? -strafeV : strafeV;
  return strafeV;
}

double Odometry::getForwardVelocity(){ 
  double fwdVelocity = (linRot.velocity(vex::velocityUnits::rpm) / 60) * 2 * M_PI * INERTIAL_WHEEL_RADIUS; 
  fwdVelocity = RobotState::getStateOf("inverted") ? -fwdVelocity : fwdVelocity; 
  return fwdVelocity; 
}

void Odometry::deriveSimpleTranslations(double& xTrans, double &yTrans, double forwardDist, double strafeDist, double heading){ 
   xTrans = cos(toRadians(heading)) * forwardDist + sin(toRadians(heading)) * strafeDist; 
   yTrans = sin(toRadians(heading)) * forwardDist + cos(toRadians(heading)) * strafeDist; 
} 

void Odometry::deriveArcTranslations(double& xTrans, double& yTrans, double forwardDist, double strafeDist, double thetaTrans, double heading){ 
  if (fabs(toRadians(thetaTrans)) < 0.001){ 
    deriveSimpleTranslations(xTrans, yTrans, forwardDist, strafeDist, heading); 
  } else { 
    double thetaTrans = toRadians(thetaTrans); 
    double radsHeading = toRadians(heading);

    double rFwd = forwardDist / thetaTrans;
    double rStr = strafeDist / thetaTrans;
    
    double xTransFwd = rFwd * sin(thetaTrans); 
    double yTransFwd = rFwd * (1 - cos(thetaTrans));  

    double xTransStr = rStr * (1 - cos(thetaTrans)); 
    double yTransStr = rStr * sin(thetaTrans);  
    
    double relX = xTransFwd;//+ xTransStr; 
    double relY = yTransFwd;// + yTransStr;
    
    xTrans = relX * cos(radsHeading) + relY * sin(radsHeading); 
    yTrans = relX * sin(radsHeading) + relY * cos(radsHeading);
  }
}

void Odometry::generateReckoningTranslations(double& xTrans, double& yTrans, double delta){ 
  double currentHeading = getProcessedHeading(); 
  double strafeVelocity = getStrafeVelocity(); 
  double fwdVelocity = getForwardVelocity(); 
  double omega = getOmega();  

  set<double>("heading_deg", currentHeading); 
  set<double>("velocity_ms", fwdVelocity); 
  set<double>("immediate_distance", fwdVelocity * delta);
  
  deriveArcTranslations(xTrans, yTrans, fwdVelocity * delta, strafeVelocity * delta, omega * delta, currentHeading);
};

void Odometry::refreshData(){  
    set<bool>("obstructed", frontDist.objectDistance(vex::distanceUnits::mm) <= 100);
    double currentTimestamp = Brain.Timer.time();  
    if (initialized){  

      double delta = (currentTimestamp - lastTimestamp) / 1000.0;   
      
      double xTrans;
      double yTrans;

      generateReckoningTranslations(xTrans, yTrans, delta);
      
      xCenterOfRot += xTrans;
      yCenterOfRot += yTrans; 

      double headingRads = toRadians(get<double>("heading_deg"));
     
      if (get<bool>("dr_reset_triggered")){
        laserCalibrate(xCenterOfRot, yCenterOfRot);
        xCenterOfRot += (cos(headingRads) * OFFSET_COR_Y + sin(headingRads) * OFFSET_COR_X);
        yCenterOfRot += (cos(headingRads) * OFFSET_COR_X + sin(headingRads) * OFFSET_COR_Y);
        set<bool>("dr_reset_triggered", false);
      }

      if (get<bool>("proxy_reset_triggered")){
       resetOffGoal(get<int>("goal_index"), xCenterOfRot, yCenterOfRot); 
       xCenterOfRot += (cos(headingRads) * OFFSET_COR_Y + sin(headingRads) * OFFSET_COR_X);
       yCenterOfRot += (cos(headingRads) * OFFSET_COR_X + sin(headingRads) * OFFSET_COR_Y);
       set<bool>("proxy_reset_triggered", false);
      }
  
      set<double>("x_position_mm", xCenterOfRot - (cos(headingRads) * OFFSET_COR_Y + sin(headingRads) * OFFSET_COR_X));
      set<double>("y_position_mm", yCenterOfRot - (cos(headingRads) * OFFSET_COR_X + sin(headingRads) * OFFSET_COR_Y));
       
      Brain.Screen.clearLine(120);
      Brain.Screen.printAt(20, 120, "X: %.2f, Y: %.2f", get<double>("x_position_mm"), get<double>("y_position_mm"));

      set<double>("tip_x_position_mm", get<double>("x_position_mm") + cos(headingRads) * (ROBOT_LENGTH_MM/2)); 
      set<double>("tip_y_position_mm", get<double>("y_position_mm") + sin(headingRads) * (ROBOT_LENGTH_MM/2));  

      if (get<bool>("ang_reset_triggered")){ 
       snapHeading();
       set<bool>("ang_reset_triggered", false);
      }

      set<bool>("opened", true);
    } else {
      calibratePerspective(); 
      initialized = true;
    }
    lastTimestamp = currentTimestamp; 
} 

void Odometry::resetOffGoal(int goal, double& xPos, double& yPos){ 
    double translationalHeading = angleSum(getProcessedHeading(), 180); 
    Location* loc = getLocation(goal); 
    array<double, 2> newRef = loc->getProjectedSetpoint(ROBOT_LENGTH_MM/2 + GOAL_WIDTH/2, translationalHeading); 
    xPos = newRef[0]; 
    yPos = newRef[1]; 
}

void Odometry::laserCalibrate(double& resX, double& resY){ 
  double xAccum = 0;
  int xCount = 0;
  double yAccum = 0;
  int yCount = 0;
  
  if (get<bool>("factor_front")){ 
     findTranslations(xAccum, yAccum, xCount, yCount, frontDist, 0, 0, (25.4 * 3.501)); 
     set<bool>("factor_front", false); 
  }  

  if (get<bool>("factor_left")){ 
     findTranslations(xAccum, yAccum, xCount, yCount, leftDist, 90, -ROBOT_WIDTH_MM/2, 0); 
     set<bool>("factor_left", false); 
  } 

  if (get<bool>("factor_right")){ 
     findTranslations(xAccum, yAccum, xCount, yCount, rightDist, 270, ROBOT_WIDTH_MM/2, 0); 
     set<bool>("factor_right", false); 
  }   

  if (get<bool>("factor_back")){
     findTranslations(xAccum, yAccum, xCount, yCount, backDist, 180, 0, -3.5 * 25.4);  
     set<bool>("factor_back", false); 
  }

  if (xCount > 0){ 
    resX = (xAccum * 1.0) / xCount;
  }  
  if (yCount > 0){ 
    resY = (yAccum * 1.0) / yCount;
  }

}

void Odometry::snapHeading(){  
   double resultantHeading;
   double rawHeading = gyro.heading(); 
   if (rawHeading > 45 && rawHeading < 135){  
     resultantHeading = 90;
   } else if (rawHeading > 135 && rawHeading < 225) { 
     resultantHeading = 180; 
   } else if (rawHeading > 225 && rawHeading < 315){ 
     resultantHeading = 270;
   } else { 
     resultantHeading = 0;
   } 
   gyro.setHeading(resultantHeading, vex::rotationUnits::deg); 
}

double Odometry::getApplicableWallHeading(double currentX, double currentY, double currentHeading, double xOffset, double yOffset){ 
  double res; 

  double headingRads = toRadians(currentHeading);  

  double sensorX = currentX + cos(headingRads) * yOffset + sin(headingRads) * xOffset; 
  double sensorY = currentY + sin(headingRads) * yOffset + cos(headingRads) * xOffset;  
  
  double topRight[2] = {TILE_SIZE_MM * 6, TILE_SIZE_MM * 6}; 
  double topLeft[2] = {0, TILE_SIZE_MM * 6};  
  double bottomRight[2] = {TILE_SIZE_MM * 6, 0}; 
  double bottomLeft[2] = {0, 0};
  //                              0:90    1:180        2:90        3:180
  //                               |        |           |            |
  double corners[4][2]; 

  memcpy(corners[0], topRight, sizeof(topRight)); 
  memcpy(corners[1], topLeft, sizeof(topLeft)); 
  memcpy(corners[2], bottomLeft, sizeof(bottomLeft)); 
  memcpy(corners[3], bottomRight, sizeof(bottomRight));
  
  int ind = -1;

  for (int i = 0; i < 4; i ++){ 
    int nextInd = i >= 3 ? 0 : i + 1;
    
    double thetaLeft = angleSum(toDegrees(atan2(corners[i][1] - sensorY, corners[i][0] - sensorX)), 0); 
    double thetaRight = angleSum(toDegrees(atan2(corners[nextInd][1] - sensorY, corners[nextInd][0] - sensorX)), 0);  
    
    if (copysign(1, angleDifference(thetaLeft, currentHeading)) != copysign(1, angleDifference(thetaRight, currentHeading))){ 
      ind = i; 
      break;
    } 
  }  

  if (ind % 2 == 0){ 
     res = 90;
  } else {
     res = 180;
  } 

  return res; 
}

void Odometry::findTranslations(double& xCoords, double& yCoords, int& xCount, int& yCount, vex::distance& sensor, double angleOffset, double xOffset, double yOffset){ 
   
   double sensorHeading = angleSum(getHeading(), angleOffset);
   double wallHeading = getApplicableWallHeading(get<double>("x_position_mm"), get<double>("y_position_mm"), sensorHeading, xOffset, yOffset);
   
   /*
   if (sensorHeading > 45 && sensorHeading < 135){ //90 + 90 -> cos(180) -> -1  
     wallHeading = 90;
   } else if (sensorHeading > 135 && sensorHeading < 225) { //180 + 180 -> cos(360) -> 1
     wallHeading = 180;
   } else if (sensorHeading > 225 && sensorHeading < 315){ //270 + 90 -> cos(360) -> 1
     wallHeading = 90;
   } else { // 0 + 180 -> cos(180) -> -1
     wallHeading = 180;
   } 
   */
   
   sensorHeading = toRadians(angleSum(sensorHeading + wallHeading, 0)); 
     
   double distance = cos(sensorHeading) * (sensor.objectDistance(vex::distanceUnits::mm) + xOffset) + sin(sensorHeading) * yOffset; 
   
   double normPosition = distance < 0 ? (TILE_SIZE_MM*6) + distance : distance;  
    
   //Brain.Screen.printAt(20, 150, "Norm Position: %.2f", normPosition); 

   if (wallHeading == 90){
      yCoords += normPosition;
      yCount++;
   } else { 
      xCoords += normPosition;
      xCount++;
   } 
}

void Odometry::calibratePerspective(){
  //double cornerX = TILE_SIZE_MM * 3; 
  xCenterOfRot = (TILE_SIZE_MM * 3);
  if (RobotState::getStateOf("starting_left")){
   for (int i = 0; i < 8; i++){ 
     Location* currentLocation = getLocation(i);  
     string name = currentLocation->getName(); 
     bool vertical = (name[0] == 'a' && name[4] == 'f') || (name[0] == 'o' && name[4] == 'n');
     if (vertical){  
       currentLocation->setY((TILE_SIZE_MM * 6) - currentLocation->getY()); 
     } 
     currentLocation->setX((TILE_SIZE_MM * 6) - currentLocation->getX()); 
   }
  } else { 
    //cornerX = TILE_SIZE_MM * 3 - ROBOT_WIDTH_MM + (2.551 * 25.4); 
    xCenterOfRot -= ROBOT_WIDTH_MM - (2.551 * 25.4); 
  }

  gyro.setHeading(90, vex::rotationUnits::deg);
  //set<double>("x_position_mm", cornerX + (ROBOT_WIDTH_MM/2.0));
  //set<double>("y_position_mm", (ROBOT_LENGTH_MM/2.0));
  yCenterOfRot = (ROBOT_LENGTH_MM/2.0); 

  xCenterOfRot += OFFSET_COR_X; 
  yCenterOfRot += OFFSET_COR_Y;

  lastTimestamp = Brain.Timer.time();
}