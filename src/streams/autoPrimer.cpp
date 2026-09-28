#include "autoPrimer.h" 

double STACK_HEIGHT_MM = 100; 

double Goal::getHeight(){ 
    return goalHeight;
}  

double Goal::getDistance(double botX, double botY){ 
    return hypot(botX - goalPosition->getX(), botY - goalPosition->getY());
} 

void Goal::setHeight(double height){ 
    goalHeight = height;
}

void AutoPrimer::init(){     

    Goal goalWrap1; 
    goalWrap1.goalPosition = Odometry::getLocation(Setpoint::ALL_NAT_NEU); 
    

    Goal goalWrap2; 
    goalWrap2.goalPosition = Odometry::getLocation(Setpoint::OPP_NAT_ALL); 
    

    Goal goalWrap3; 
    goalWrap3.goalPosition = Odometry::getLocation(Setpoint::OPP_FOR_ALL); 
    

    Goal goalWrap4; 
    goalWrap4.goalPosition = Odometry::getLocation(Setpoint::ALL_FOR_ALL); 
    
    
    Goal goalWrap5; 
    goalWrap5.goalPosition = Odometry::getLocation(Setpoint::ALL_FOR_ALL); 
    

    Goal goalWrap6; 
    goalWrap6.goalPosition = Odometry::getLocation(Setpoint::ALL_FOR_ALL); 
    

    Goal goalWrap7; 
    goalWrap7.goalPosition = Odometry::getLocation(Setpoint::ALL_FOR_ALL); 
    

    Goal goalWrap8; 
    goalWrap8.goalPosition = Odometry::getLocation(Setpoint::ALL_FOR_ALL); 
    

    goals[0] = goalWrap1; 
    goals[1] = goalWrap2; 
    goals[2] = goalWrap3;
    goals[3] = goalWrap4; 
    goals[4] = goalWrap5; 
    goals[5] = goalWrap6; 
    goals[6] = goalWrap7; 
    goals[7] = goalWrap8; 
} 

void AutoPrimer::refreshData(){ 
   int pos = Telemetry::inst.getValueAt<int>("ss_manager", "position"); 
   bool indexChanged = updateGoalIndex();  
   if (get<bool>("active")){ 
     if (indexChanged && pos == SuperStructurePosition::PRIMED && Telemetry::inst.getValueAt<bool>("claw", "in_possession")){ 
      pasteSetpoints();
     } else if (pos == SuperStructurePosition::AUTO && !Telemetry::inst.getValueAt<bool>("elevator", "active")){ 
      replaceGoalHeight();
     }
   }
   Brain.Screen.printAt(20, 120, "Goal Index: %d", goalIndex);   
   
}

void AutoPrimer::locationBasedShiftUpdate(){   
  
   double botX = Telemetry::inst.getValueAt<double>("odometry", "x_position_mm"); 
   double botY = Telemetry::inst.getValueAt<double>("odometry", "y_position_mm"); 
   
   int chosenGoalIndex = goalIndex; 
   double closestDistance = goals[goalIndex].getDistance(botX, botY);  
   
   for (int i = 0; i < 8; i++){   
    if (i == goalIndex){ 
        continue;
    }
    double currentDist = goals[i].getDistance(botX, botY);  
    if (currentDist < closestDistance){ 
        chosenGoalIndex = i; 
        closestDistance = currentDist;
    }   
   }

   goalIndex = chosenGoalIndex;
}  

void AutoPrimer::replaceGoalHeight(){  
   if (goalIndex < 0){ 
       return;
    }
   goals[goalIndex].setHeight(Telemetry::inst.getValueAt<double>("elevator", "current_height"));
}

void AutoPrimer::manualBasedShiftUpdate(){ 
   int shift = get<int>("shift_direction"); 
   if (shift != 0){ 
    goalIndex += shift;
    if (goalIndex == 8){ 
        goalIndex = 0;
    } else if (goalIndex == -1){ 
        goalIndex = 7;
    } 
    set<int>("shift_direction", 0); 
   }   
}

void AutoPrimer::pasteSetpoints(){  
    if (goalIndex < 0){ 
       return;
    }
    Telemetry::inst.placeValueAt<bool>(true, "elevator", "sniper_score_enabled");
    Telemetry::inst.placeValueAt<bool>(true, "elevator", "requesting_setpoint"); 
    Telemetry::inst.placeValueAt<double>(goals[goalIndex].getHeight(), "elevator", "requested_setpoint"); 
    lastGoalIndex = goalIndex;
}

bool AutoPrimer::updateGoalIndex(){    
    locationBasedShiftUpdate();
    /*
    if (get<int>("priming_method") == DestinationProtocol::LOCATION || (!get<bool>("active"))){ 
        locationBasedShiftUpdate();
    } else { 
        manualBasedShiftUpdate();
    }  
    */
    return goalIndex == lastGoalIndex; 
}

